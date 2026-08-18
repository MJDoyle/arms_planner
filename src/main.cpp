#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/GlbWriter.hpp"
#include "assembler/Logger.hpp"
#include "assembler/ModelLoader.hpp"
#include "assembler/Part.hpp"
#include "assembler/VacuumGraspGenerator.hpp"

#include <boost/json.hpp>
#include <minizip/zip.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace json = boost::json;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::string iso8601_now()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc{};
    gmtime_r(&t, &tm_utc);
    std::ostringstream ss;
    ss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return ss.str();
}

static std::string pipeline_commit()
{
    // Run git to get the current commit of the arms_planner repo.
    FILE* p = popen("git -C \"$(dirname \"$(readlink -f /proc/self/exe)\")\" "
                    "rev-parse --short HEAD 2>/dev/null", "r");
    if (!p) return "unknown";
    char buf[64] = {};
    if (!fgets(buf, sizeof(buf), p)) { pclose(p); return "unknown"; }
    pclose(p);
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s.empty() ? "unknown" : s;
}

static const char* part_type_str(Part::PART_TYPE t)
{
    switch (t) {
        case Part::INTERNAL: return "internal";
        case Part::EXTERNAL: return "external";
        case Part::SCREW:    return "screw";
        default:             return "unknown";
    }
}

static const char* grasp_status_str(GraspAttempt::Status s)
{
    switch (s) {
        case GraspAttempt::Status::body_collision:     return "body_collision";
        case GraspAttempt::Status::assembly_collision: return "assembly_collision";
        case GraspAttempt::Status::seal_failed:        return "seal_failed";
        case GraspAttempt::Status::accepted:           return "accepted";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// .arms zip writer
// ---------------------------------------------------------------------------

class ArmsZipWriter
{
public:
    explicit ArmsZipWriter(const std::string& path)
    {
        zf_ = zipOpen(path.c_str(), APPEND_STATUS_CREATE);
        if (!zf_) throw std::runtime_error("Cannot create .arms file: " + path);
    }

    ~ArmsZipWriter() { if (zf_) zipClose(zf_, nullptr); }

    void add(const std::string& entry_name, const void* data, size_t size)
    {
        zip_fileinfo zi{};
        int err = zipOpenNewFileInZip(zf_, entry_name.c_str(), &zi,
                                      nullptr, 0, nullptr, 0, nullptr,
                                      Z_DEFLATED, Z_DEFAULT_COMPRESSION);
        if (err != ZIP_OK)
            throw std::runtime_error("zipOpenNewFileInZip failed for: " + entry_name);
        zipWriteInFileInZip(zf_, data, static_cast<unsigned>(size));
        zipCloseFileInZip(zf_);
    }

    void add(const std::string& entry_name, const std::string& text)
    {
        add(entry_name, text.data(), text.size());
    }

    void add(const std::string& entry_name, const std::vector<uint8_t>& bytes)
    {
        add(entry_name, bytes.data(), bytes.size());
    }

private:
    zipFile zf_ = nullptr;
};

// ---------------------------------------------------------------------------
// Manifest builder
// ---------------------------------------------------------------------------

static json::object build_manifest(
    const std::string&                              source_cad,
    const std::string&                              commit,
    const std::string&                              timestamp,
    const std::shared_ptr<Assembly>&                target,
    const std::shared_ptr<Assembly>&                initial,
    const std::vector<std::shared_ptr<AssemblyNode>>& path,
    const std::vector<GraspAttempt>&                grasps,
    const std::string&                              run_output_dir)
{
    json::object manifest;
    manifest["version"]         = "1";
    manifest["source_cad"]      = source_cad;
    manifest["generated"]       = timestamp;
    manifest["pipeline_commit"] = commit;

    // ---- parts ----
    json::array parts_arr;
    // Collect all parts from target (assembled positions) and initial (bay positions)
    for (auto const& [part, assembled_pos] : target->getAssembledPartTransforms())
    {
        json::object p;
        p["part_id"] = static_cast<int64_t>(part->getId());
        p["name"]    = part->getName();
        p["type"]    = part_type_str(part->getType());
        p["mesh_file"] = "part_" + std::to_string(part->getId()) + ".glb";

        json::object apose;
        apose["x"] = assembled_pos.X();
        apose["y"] = assembled_pos.Y();
        apose["z"] = assembled_pos.Z();
        p["assembled_pose"] = apose;

        // Bay pose from initial assembly
        json::object bpose;
        bpose["x"] = json::value(0.0);
        bpose["y"] = json::value(0.0);
        bpose["z"] = json::value(0.0);
        if (initial)
        {
            for (auto const& [ip, ipos] : initial->getUnassembledPartTransforms())
            {
                if (ip->getId() == part->getId())
                {
                    bpose["x"] = ipos.X();
                    bpose["y"] = ipos.Y();
                    bpose["z"] = ipos.Z();
                    break;
                }
            }
        }
        p["bay_pose"] = bpose;

        parts_arr.push_back(std::move(p));
    }
    manifest["parts"] = std::move(parts_arr);

    // ---- stages ----
    json::array stages_arr;
    for (size_t i = 0; i < path.size(); ++i)
    {
        auto const& node = path[i];
        json::object stage;
        stage["step"] = static_cast<int64_t>(i);

        json::array assembled_ids;
        for (auto const& [part, _] : node->assembly_->getAssembledPartTransforms())
            assembled_ids.push_back(static_cast<int64_t>(part->getId()));
        stage["assembled_part_ids"] = std::move(assembled_ids);

        json::array unassembled_ids;
        for (auto const& [part, _] : node->assembly_->getUnassembledPartTransforms())
            unassembled_ids.push_back(static_cast<int64_t>(part->getId()));
        stage["unassembled_part_ids"] = std::move(unassembled_ids);

        if (node->edge_part_)
        {
            stage["edge_part_id"] = static_cast<int64_t>(node->edge_part_->getId());
            json::object eg;
            eg["x"] = node->edge_grasp_.X();
            eg["y"] = node->edge_grasp_.Y();
            eg["z"] = node->edge_grasp_.Z();
            stage["edge_grasp"] = std::move(eg);
        }

        stages_arr.push_back(std::move(stage));
    }
    manifest["stages"] = std::move(stages_arr);

    // ---- grasps ----
    json::array grasps_arr;
    for (auto const& g : grasps)
    {
        json::object ga;
        ga["x_mm"]  = g.x_mm;
        ga["y_mm"]  = g.y_mm;
        ga["z_mm"]  = g.z_mm;
        ga["status"] = grasp_status_str(g.status);
        grasps_arr.push_back(std::move(ga));
    }
    manifest["grasps"] = std::move(grasps_arr);

    // ---- jigs ----
    // List jig STLs that exist in the output dir
    json::array jigs_arr;
    if (!run_output_dir.empty() && fs::is_directory(run_output_dir))
    {
        for (auto const& entry : fs::directory_iterator(run_output_dir))
        {
            if (entry.path().extension() == ".stl")
            {
                std::string fname = entry.path().filename().string();
                if (fname.rfind("jig_", 0) == 0)
                {
                    json::object jig;
                    jig["stl_file"] = fname;
                    jigs_arr.push_back(std::move(jig));
                }
            }
        }
    }
    manifest["jigs"] = std::move(jigs_arr);

    manifest["gcode_file"]   = "assembler.gcode";
    manifest["command_file"] = "assembly_plan.yaml";

    return manifest;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static void print_usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " --input <model.step> [options]\n"
        << "\n"
        << "Options:\n"
        << "  --input  <path>            STEP file to process (required)\n"
        << "  --output <path>            Output .arms file (default: <model_stem>.arms)\n"
        << "  --output-dir <dir>         Pipeline output directory (default: arms_output/<stem>/)\n"
        << "  --slicer-config-dir <dir>  Directory containing arms_prusa_config.ini\n"
        << "                             (if omitted and config not found, slicer step is skipped)\n"
        << "  --no-grasps                Skip grasp generation\n"
        << "  --no-jigs                  Skip jig STL generation\n"
        << "  --no-path                  Skip path planning\n";
}

int main(int argc, char* argv[])
{
    // Install default log sink (stderr)
    g_arms_log_sink = [](int level, const std::string& msg) {
        const char* prefix = "INFO";
        if      (level >= arms_log::FATAL) prefix = "FATAL";
        else if (level >= arms_log::ERROR) prefix = "ERROR";
        else if (level >= arms_log::WARN)  prefix = "WARN";
        else if (level <= arms_log::DEBUG) prefix = "DEBUG";
        std::fprintf(stderr, "[arms/%s] %s\n", prefix, msg.c_str());
    };

    std::string input_path;
    std::string output_path;
    std::string output_dir;
    // Default to the config/ directory baked in at build time.
    std::string slicer_config_dir = std::string(ARMS_PLANNER_CONFIG_DIR) + "/";
    bool generate_grasps = true;
    bool generate_jigs   = true;
    bool generate_path   = true;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if      (arg == "--input"             && i + 1 < argc) { input_path        = argv[++i]; }
        else if (arg == "--output"            && i + 1 < argc) { output_path       = argv[++i]; }
        else if (arg == "--output-dir"        && i + 1 < argc) { output_dir        = argv[++i]; }
        else if (arg == "--slicer-config-dir" && i + 1 < argc) { slicer_config_dir = argv[++i]; }
        else if (arg == "--no-grasps") { generate_grasps = false; }
        else if (arg == "--no-jigs")   { generate_jigs   = false; }
        else if (arg == "--no-path")   { generate_path   = false; }
        else if (arg == "--help" || arg == "-h") { print_usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << arg << "\n"; print_usage(argv[0]); return 1; }
    }

    if (input_path.empty()) { print_usage(argv[0]); return 1; }
    if (!fs::exists(input_path))
    {
        std::cerr << "ERROR: input file not found: " << input_path << "\n";
        return 1;
    }

    const std::string model_stem = fs::path(input_path).stem().string();

    if (output_path.empty())
        output_path = model_stem + ".arms";

    if (output_dir.empty())
        output_dir = "arms_output/" + model_stem + "/";

    fs::create_directories(output_dir);
    if (!output_dir.empty() && output_dir.back() != '/')
        output_dir += '/';

    std::fprintf(stderr, "[arms-plan] input:      %s\n", input_path.c_str());
    std::fprintf(stderr, "[arms-plan] output:     %s\n", output_path.c_str());
    std::fprintf(stderr, "[arms-plan] output-dir: %s\n", output_dir.c_str());

    try
    {
        // ---- Load model ----
        std::fprintf(stderr, "[arms-plan] Loading model...\n");
        auto target_assembly = ModelLoader::loadModel(input_path);

        // ---- Configure and run assembler ----
        Assembler assembler;
        assembler.setName(model_stem);
        assembler.setRunOutputDir(output_dir);
        assembler.setGenerateGrasps(generate_grasps);
        assembler.setGenerateJigs(generate_jigs);
        assembler.setGeneratePath(generate_path);
        assembler.setTargetAssembly(target_assembly);
        if (!slicer_config_dir.empty())
            assembler.setSlicerConfigDir(slicer_config_dir);

        std::fprintf(stderr, "[arms-plan] Running pipeline...\n");
        assembler.generateAssemblySequence();

        // ---- Collect results ----
        auto initial = assembler.getInitialAssembly();
        auto path    = assembler.getAssemblyPath();

        // All grasp attempts (requires generate_grasps)
        std::vector<GraspAttempt> grasps;
        if (generate_grasps)
        {
            std::fprintf(stderr, "[arms-plan] Collecting grasp debug data...\n");
            grasps = assembler.debugGrasps();
        }

        // ---- Build manifest ----
        std::fprintf(stderr, "[arms-plan] Building manifest...\n");
        const std::string timestamp = iso8601_now();
        const std::string commit    = pipeline_commit();

        json::object manifest = build_manifest(
            fs::absolute(input_path).string(),
            commit, timestamp,
            target_assembly, initial, path,
            grasps, output_dir);

        const std::string manifest_str = json::serialize(manifest);

        // ---- Write .arms zip ----
        std::fprintf(stderr, "[arms-plan] Writing %s...\n", output_path.c_str());
        ArmsZipWriter zip(output_path);
        zip.add("manifest.json", manifest_str);

        // Add per-part GLB meshes
        size_t glb_count = 0;
        for (auto const& [part, _] : target_assembly->getAssembledPartTransforms())
        {
            auto mesh = part->get_mesh_asset();
            if (!mesh || mesh->vertices.empty()) continue;
            auto glb_bytes = mesh_to_glb(*mesh);
            zip.add("part_" + std::to_string(part->getId()) + ".glb", glb_bytes);
            ++glb_count;
        }

        std::fprintf(stderr, "[arms-plan] Done. %zu parts, %zu stages, %zu grasps, %zu GLBs → %s\n",
                     target_assembly->getAssembledPartTransforms().size(),
                     path.size(), grasps.size(), glb_count,
                     output_path.c_str());
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "[arms-plan] FATAL: %s\n", e.what());
        return 1;
    }

    return 0;
}
