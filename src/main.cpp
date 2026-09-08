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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <STEPControl_Reader.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Ax1.hxx>

#include "assembler/MeshAsset.hpp"
#include "assembler/MeshFunctions.hpp"

namespace fs = std::filesystem;
namespace json = boost::json;

// ---------------------------------------------------------------------------
// Background model helpers
// ---------------------------------------------------------------------------

struct BackgroundEntry {
    std::string path;
    float r = 0.5f, g = 0.5f, b = 0.5f;
    double tx = 0, ty = 0, tz = 0;    // mm
    double qx = 0, qy = 0, qz = 0, qw = 1;
};

// Parse "path,r,g,b,x,y,z,qx,qy,qz,qw" — same format as the ROS2 background_models param.
static std::optional<BackgroundEntry> parse_background_entry(const std::string& s)
{
    std::vector<std::string> parts;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ','))
        parts.push_back(tok);

    if (parts.size() < 11) return std::nullopt;

    BackgroundEntry e;
    try {
        const size_t n = parts.size();
        e.qw = std::stod(parts[n - 1]);
        e.qz = std::stod(parts[n - 2]);
        e.qy = std::stod(parts[n - 3]);
        e.qx = std::stod(parts[n - 4]);
        e.tz = std::stod(parts[n - 5]);
        e.ty = std::stod(parts[n - 6]);
        e.tx = std::stod(parts[n - 7]);
        e.b  = std::stof(parts[n - 8]);
        e.g  = std::stof(parts[n - 9]);
        e.r  = std::stof(parts[n - 10]);
        // Everything before the last 10 fields is the path (handles commas in path).
        e.path = parts[0];
        for (size_t i = 1; i < n - 10; ++i)
            e.path += "," + parts[i];
    } catch (...) {
        return std::nullopt;
    }
    return e;
}

// Load an STL file (binary or ASCII) and return a MeshAsset with vertices
// already transformed to world-space metres via the given pose.
// Transform: p_world_m = 0.001 * (R(q) * p_mm + t_mm)
static MeshAsset load_stl_world_space(const BackgroundEntry& e)
{
    const double s  = 0.001;
    const double qx = e.qx, qy = e.qy, qz = e.qz, qw = e.qw;
    const double R[3][3] = {
        {1 - 2*(qy*qy+qz*qz),  2*(qx*qy-qz*qw),  2*(qx*qz+qy*qw)},
        {2*(qx*qy+qz*qw),      1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)},
        {2*(qx*qz-qy*qw),      2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)}
    };

    auto xform = [&](float px, float py, float pz, double& wx, double& wy, double& wz) {
        wx = s * (R[0][0]*px + R[0][1]*py + R[0][2]*pz + e.tx);
        wy = s * (R[1][0]*px + R[1][1]*py + R[1][2]*pz + e.ty);
        wz = s * (R[2][0]*px + R[2][1]*py + R[2][2]*pz + e.tz);
    };

    MeshAsset mesh;
    std::ifstream f(e.path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "[arms-plan] WARNING: cannot open background STL: %s\n", e.path.c_str());
        return mesh;
    }

    char header[80]{};
    f.read(header, 80);
    if (!f) return mesh;

    uint32_t n_tris = 0;
    f.read(reinterpret_cast<char*>(&n_tris), 4);
    if (!f) return mesh;

    // Check if this is a binary STL (file size must equal 84 + n_tris * 50).
    const std::uintmax_t expected = 84u + static_cast<std::uintmax_t>(n_tris) * 50u;
    const bool is_binary = (fs::file_size(e.path) == expected);

    if (is_binary) {
        mesh.vertices.reserve(n_tris * 3);
        mesh.triangles.reserve(n_tris);
        for (uint32_t i = 0; i < n_tris; ++i) {
            float nrm[3];
            f.read(reinterpret_cast<char*>(nrm), 12);
            const int base = static_cast<int>(mesh.vertices.size());
            for (int v = 0; v < 3; ++v) {
                float vx, vy, vz;
                f.read(reinterpret_cast<char*>(&vx), 4);
                f.read(reinterpret_cast<char*>(&vy), 4);
                f.read(reinterpret_cast<char*>(&vz), 4);
                double wx, wy, wz;
                xform(vx, vy, vz, wx, wy, wz);
                mesh.vertices.push_back({wx, wy, wz});
            }
            mesh.triangles.push_back({base, base + 1, base + 2});
            uint16_t attr;
            f.read(reinterpret_cast<char*>(&attr), 2);
        }
    } else {
        // ASCII STL: scan for "vertex px py pz" lines.
        f.clear();
        f.seekg(0);
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::string word;
            iss >> word;
            if (word == "vertex") {
                float px, py, pz;
                if (iss >> px >> py >> pz) {
                    double wx, wy, wz;
                    xform(px, py, pz, wx, wy, wz);
                    mesh.vertices.push_back({wx, wy, wz});
                }
            }
        }
        for (size_t i = 0; i + 2 < mesh.vertices.size(); i += 3)
            mesh.triangles.push_back({static_cast<int>(i),
                                      static_cast<int>(i) + 1,
                                      static_cast<int>(i) + 2});
    }

    return mesh;
}

// A hinge axis running along the tool's Y, expressed in the re-based frame.
struct Pivot { double x = 0.0, z = 0.0; };

// One arm of the jaw linkage: it hinges on the chassis and carries a jaw.
struct LinkInfo {
    size_t    solid = 0;
    Pivot     body;   // fixed end
    Pivot     jaw;    // moving end
    MeshAsset mesh;
    double length() const { return std::hypot(jaw.x - body.x, jaw.z - body.z); }
};


// ---------------------------------------------------------------------------
// Tool models
//
// The tools are supplied as STEP assemblies in their own design origin.  Each is
// re-based on the centre of its lowest-z geometry — the nozzle tip, or the
// underside of the jaw carrier — because that is the point the planner's grasp
// coordinates refer to.
//
// The gripper is split so its jaws can be drawn at the opening the plan actually
// calls for rather than the one it happens to be modelled at.  The two jaws are
// identified geometrically, not by index: they are the pair of solids that are
// thin along the jaw axis, mirror-symmetric about the tool centreline, and reach
// down near the tool's lowest point.  If no such pair is found the tool is baked
// as a single body and drawn at its modelled opening.
// ---------------------------------------------------------------------------

struct ToolMesh {
    MeshAsset mesh;
    double    inner_mm  = 0.0;  // jaws only: distance from centreline to the gripping face
    double    jaw_z0_mm = 0.0;  // jaws only: height of the jaw underside above the tool origin
    std::vector<Pivot>    pivots;   // jaws only
    std::vector<LinkInfo> links;    // arms carrying this jaw
};

static bool load_tool_shape(const std::string& path, TopoDS_Shape& out)
{
    STEPControl_Reader reader;
    if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) return false;
    reader.TransferRoots();
    out = reader.OneShape();
    return !out.IsNull();
}

// Centre of the geometry sitting at the shape's lowest z.
static gp_Pnt lowest_face_centre(const TopoDS_Shape& shape)
{
    BRepMesh_IncrementalMesh mesher(shape, 0.5);
    mesher.Perform();

    std::vector<gp_Pnt> pts;
    double lo = 1e18;
    for (TopExp_Explorer e(shape, TopAbs_FACE); e.More(); e.Next()) {
        TopLoc_Location L;
        Handle(Poly_Triangulation) t = BRep_Tool::Triangulation(TopoDS::Face(e.Current()), L);
        if (t.IsNull()) continue;
        const gp_Trsf T = L.IsIdentity() ? gp_Trsf() : L.Transformation();
        for (int i = 1; i <= t->NbNodes(); ++i) {
            const gp_Pnt p = t->Node(i).Transformed(T);
            pts.push_back(p);
            lo = std::min(lo, p.Z());
        }
    }
    double sx = 0, sy = 0; int n = 0;
    for (auto const& p : pts)
        if (p.Z() < lo + 0.5) { sx += p.X(); sy += p.Y(); ++n; }
    if (n == 0) return gp_Pnt(0, 0, 0);
    return gp_Pnt(sx / n, sy / n, lo);
}

// Hinge axes of a solid: cylindrical faces whose axis runs along Y, which is the
// only axis a jaw linkage can turn about if the jaws move in the x-z plane.
static std::vector<Pivot> pivots_of(const TopoDS_Shape& solid, double ox, double oz)
{
    std::vector<Pivot> out;
    for (TopExp_Explorer f(solid, TopAbs_FACE); f.More(); f.Next()) {
        BRepAdaptor_Surface su(TopoDS::Face(f.Current()));
        if (su.GetType() != GeomAbs_Cylinder) continue;
        const gp_Cylinder cy = su.Cylinder();
        if (std::abs(cy.Axis().Direction().Y()) < 0.95) continue;
        const gp_Pnt p = cy.Axis().Location();
        const Pivot q{p.X() - ox, p.Z() - oz};
        bool dup = false;
        for (auto const& e : out)
            if (std::hypot(e.x - q.x, e.z - q.z) < 0.4) { dup = true; break; }
        if (!dup) out.push_back(q);
    }
    return out;
}

static bool same_pivot(const Pivot& a, const Pivot& b)
{
    return std::hypot(a.x - b.x, a.z - b.z) < 0.6;
}

static MeshAsset tessellate_group(const std::vector<TopoDS_Shape>& solids, double deflection)
{
    MeshAsset out;
    for (auto const& s : solids) {
        BRepMesh_IncrementalMesh mesher(s, deflection);
        mesher.Perform();
        for (TopExp_Explorer e(s, TopAbs_FACE); e.More(); e.Next()) {
            const TopoDS_Face& f = TopoDS::Face(e.Current());
            TopLoc_Location L;
            Handle(Poly_Triangulation) t = BRep_Tool::Triangulation(f, L);
            if (t.IsNull()) continue;
            const gp_Trsf T = L.IsIdentity() ? gp_Trsf() : L.Transformation();
            const int base = static_cast<int>(out.vertices.size());
            for (int i = 1; i <= t->NbNodes(); ++i) {
                const gp_Pnt p = t->Node(i).Transformed(T);
                out.vertices.push_back({p.X() * 0.001, p.Y() * 0.001, p.Z() * 0.001});
            }
            const bool rev = (f.Orientation() == TopAbs_REVERSED);
            for (int i = 1; i <= t->NbTriangles(); ++i) {
                int a, b, c; t->Triangle(i).Get(a, b, c);
                if (rev) std::swap(b, c);
                out.triangles.push_back({base + a - 1, base + b - 1, base + c - 1});
            }
        }
    }
    return out;
}

// Bake one tool: re-base it on its lowest-face centre and split off the jaws.
// `jaw_axis_y` selects which horizontal axis the jaws separate along in the tool's
// own frame — y for this gripper.
static bool bake_tool(const std::string& path, bool split_jaws,
                      ToolMesh& body, ToolMesh& jaw_lo, ToolMesh& jaw_hi,
                      double deflection = 0.35)
{
    TopoDS_Shape shape;
    if (!load_tool_shape(path, shape)) return false;

    // The tools are modelled lying down — their long axis runs along the design
    // Y, and the tool-changer plate's normal along Z.  Stand them up so the long
    // axis is vertical and the coupling plate faces sideways, which also puts the
    // working end (cup, jaws, nozzle) at the bottom.
    {
        gp_Trsf upright;
        upright.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), M_PI / 2.0);
        shape = BRepBuilderAPI_Transform(shape, upright, true).Shape();
    }

    // Re-base: the origin becomes the centre of the lowest geometry.
    const gp_Pnt org = lowest_face_centre(shape);
    gp_Trsf rebase;
    rebase.SetTranslation(gp_Vec(-org.X(), -org.Y(), -org.Z()));
    shape = BRepBuilderAPI_Transform(shape, rebase, true).Shape();

    std::vector<TopoDS_Shape> solids;
    for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next())
        solids.push_back(e.Current());
    if (solids.empty()) solids.push_back(shape);

    if (!split_jaws) {
        body.mesh = tessellate_group(solids, deflection);
        return !body.mesh.vertices.empty();
    }

    Bnd_Box whole; BRepBndLib::Add(shape, whole);
    Standard_Real X0, Y0, Z0, X1, Y1, Z1; whole.Get(X0, Y0, Z0, X1, Y1, Z1);

    // The jaws separate along the tool's X once it is stood upright.
    struct Cand { size_t i; double lo, hi, thick, zlo; };
    std::vector<Cand> cands;
    const double span_x = X1 - X0;
    for (size_t i = 0; i < solids.size(); ++i) {
        Bnd_Box b; BRepBndLib::Add(solids[i], b);
        Standard_Real a0, b0, c0, a1, b1, c1; b.Get(a0, b0, c0, a1, b1, c1);
        const double thick = a1 - a0;
        // A jaw is slender across the opening axis and reaches the tool's base.
        if (thick < 0.35 * span_x && c0 < Z0 + 8.0)
            cands.push_back({i, a0, a1, thick, c0});
    }

    // Pick the mirror-symmetric pair: their inner faces should be equidistant
    // from the centreline (y = 0 after re-basing).
    long best_a = -1, best_b = -1; double best_err = 1e9;
    for (size_t p = 0; p < cands.size(); ++p)
        for (size_t q = p + 1; q < cands.size(); ++q) {
            const double inner_p = std::min(std::abs(cands[p].lo), std::abs(cands[p].hi));
            const double inner_q = std::min(std::abs(cands[q].lo), std::abs(cands[q].hi));
            const bool opposite = (cands[p].lo + cands[p].hi) * (cands[q].lo + cands[q].hi) < 0;
            if (!opposite) continue;
            const double err = std::abs(inner_p - inner_q);
            if (err < best_err) { best_err = err; best_a = p; best_b = q; }
        }

    if (best_a < 0 || best_err > 2.0) {
        std::fprintf(stderr, "[arms-plan] %s: no symmetric jaw pair found — "
                     "baking as one body\n", path.c_str());
        body.mesh = tessellate_group(solids, deflection);
        return !body.mesh.vertices.empty();
    }

    const Cand& ca = cands[best_a];
    const Cand& cb = cands[best_b];
    const size_t ia = (ca.lo + ca.hi) < 0 ? ca.i : cb.i;   // jaw on the -x side
    const size_t ib = (ca.lo + ca.hi) < 0 ? cb.i : ca.i;   // jaw on the +x side
    const Cand& la = (ca.lo + ca.hi) < 0 ? ca : cb;
    const Cand& lb = (ca.lo + ca.hi) < 0 ? cb : ca;

    std::vector<TopoDS_Shape> rest;
    for (size_t i = 0; i < solids.size(); ++i)
        if (i != ia && i != ib) rest.push_back(solids[i]);

    // ---- linkage ----
    // A parallel-plate gripper of this type carries each jaw on a pair of equal,
    // parallel arms hinged to the chassis — a parallelogram four-bar.  That is
    // what keeps the jaw faces parallel as they close, and it is also why the
    // jaws rise: the jaw end of each arm travels on a circle, so closing swings
    // it inward and upward together.
    //
    // The arms are found from the hinge axes rather than assumed: an arm is a
    // solid sharing one axis with a jaw and having a second axis nearer the
    // centreline, which is its fixed end.
    jaw_lo.pivots = pivots_of(solids[ia], 0.0, 0.0);
    jaw_hi.pivots = pivots_of(solids[ib], 0.0, 0.0);

    // The chassis is whatever carries three or more hinge axes without being a
    // jaw — its axes are the fixed ends of the arms.  Only axes down at jaw level
    // count; the drive mechanism higher up has hinges of its own.
    double jaw_top = 0.0;
    for (auto const& q : jaw_lo.pivots) jaw_top = std::max(jaw_top, q.z);
    for (auto const& q : jaw_hi.pivots) jaw_top = std::max(jaw_top, q.z);

    std::vector<Pivot> all_jaw_pivots = jaw_lo.pivots;
    all_jaw_pivots.insert(all_jaw_pivots.end(),
                          jaw_hi.pivots.begin(), jaw_hi.pivots.end());

    std::vector<Pivot> fixed;
    for (size_t i = 0; i < solids.size(); ++i) {
        if (i == ia || i == ib) continue;
        const auto pv = pivots_of(solids[i], 0.0, 0.0);
        if (pv.size() < 3) continue;
        // An arm also carries several axes, so anything hinged to a jaw is a
        // moving part, not chassis — without this the arms nominate their own
        // hinges as ground and the linkage solves to the wrong bars.
        bool touches_jaw = false;
        for (auto const& q : pv)
            for (auto const& j : all_jaw_pivots)
                if (same_pivot(q, j)) touches_jaw = true;
        if (touches_jaw) continue;

        for (auto const& q : pv)
            if (q.z <= jaw_top + 10.0) fixed.push_back(q);
    }

    // An arm bridges one chassis axis and one jaw axis.  Arms carry extra
    // cylindrical features — lightening holes, fillets — so they are matched by
    // which axes they share, not by how many they have.
    for (int side = 0; side < 2; ++side) {
        const size_t jaw_i = side == 0 ? ia : ib;
        const auto& jp = side == 0 ? jaw_lo.pivots : jaw_hi.pivots;
        auto& links    = side == 0 ? jaw_lo.links  : jaw_hi.links;

        for (size_t i = 0; i < solids.size(); ++i) {
            if (i == ia || i == ib) continue;
            const auto pv = pivots_of(solids[i], 0.0, 0.0);

            for (auto const& at_jaw : pv) {
                bool on_jaw = false;
                for (auto const& q : jp) if (same_pivot(q, at_jaw)) on_jaw = true;
                if (!on_jaw) continue;

                for (auto const& at_body : pv) {
                    if (same_pivot(at_body, at_jaw)) continue;
                    bool on_chassis = false;
                    for (auto const& f : fixed) if (same_pivot(f, at_body)) on_chassis = true;
                    if (!on_chassis) continue;
                    if (std::abs(at_body.x) >= std::abs(at_jaw.x)) continue;
                    links.push_back({i, at_body, at_jaw});
                    goto next_solid;
                }
            }
            next_solid: ;
        }
        (void)jaw_i;
    }

    // Everything on a link is carried by it, so drop those from the static body.
    std::vector<size_t> moving{ia, ib};
    for (auto const& l : jaw_lo.links) moving.push_back(l.solid);
    for (auto const& l : jaw_hi.links) moving.push_back(l.solid);
    rest.clear();
    for (size_t i = 0; i < solids.size(); ++i)
        if (std::find(moving.begin(), moving.end(), i) == moving.end())
            rest.push_back(solids[i]);

    body.mesh   = tessellate_group(rest, deflection);
    jaw_lo.mesh = tessellate_group({solids[ia]}, deflection);
    jaw_hi.mesh = tessellate_group({solids[ib]}, deflection);
    for (auto& l : jaw_lo.links) l.mesh = tessellate_group({solids[l.solid]}, deflection);
    for (auto& l : jaw_hi.links) l.mesh = tessellate_group({solids[l.solid]}, deflection);
    jaw_lo.inner_mm = std::abs(la.hi);   // inner face of the -x jaw
    jaw_hi.inner_mm = std::abs(lb.lo);   // inner face of the +x jaw
    // Where the gripping faces start, so the viewer can line the model up with
    // the planner's jaw underside rather than with the tool's lowest point.
    jaw_lo.jaw_z0_mm = std::min(la.zlo, lb.zlo);
    jaw_hi.jaw_z0_mm = jaw_lo.jaw_z0_mm;

    std::fprintf(stderr,
                 "[arms-plan] %s: jaws split, modelled opening %.2f mm; "
                 "linkage %zu + %zu arm(s), length %.2f mm\n",
                 path.c_str(), jaw_lo.inner_mm + jaw_hi.inner_mm,
                 jaw_lo.links.size(), jaw_hi.links.size(),
                 jaw_lo.links.empty() ? 0.0 : jaw_lo.links.front().length());
    return true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Read a numeric field that may be written as an int or a double.
static double jnum_or(const json::object& o, const char* key, double fallback = 0.0)
{
    if (!o.contains(key)) return fallback;
    auto const& v = o.at(key);
    if (v.is_double()) return v.as_double();
    if (v.is_int64())  return static_cast<double>(v.as_int64());
    if (v.is_uint64()) return static_cast<double>(v.as_uint64());
    return fallback;
}

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
        ga["part_id"] = static_cast<int64_t>(g.part_id);
        ga["chosen"]  = g.chosen;
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
        << "  --background-model <spec>  Background mesh to bake into the scene.\n"
        << "                             Format: \"path,r,g,b,x,y,z,qx,qy,qz,qw\"\n"
        << "                             (pose in mm + quaternion; same format as the\n"
        << "                              ROS2 background_models parameter).\n"
        << "                             May be repeated for multiple meshes.\n"
        << "  --jig-clearance <mm>       Running clearance in a jig pocket (default 0.2).\n"
        << "                             The part's convex hull is grown by this much\n"
        << "                             across its largest horizontal dimension before\n"
        << "                             being subtracted, so roughly half of it ends up\n"
        << "                             as a gap on each side of that axis.\n"
        << "  --max-splits <n>           Max printed-part splits the planner may use to\n"
        << "                             free a trapped part (default 2, 0 disables)\n"
        << "  --vacuum-model <file>      STEP model of the vacuum tool, drawn at\n"
        << "                             vacuum grasps in the viewer\n"
        << "  --gripper-model <file>     STEP model of the parallel-plate tool\n"
        << "  --grasp-overrides <file>   JSON array of hand-edited grasps to apply\n"
        << "                             instead of synthesising them (written by the\n"
        << "                             viewer's edit mode)\n"
        << "  --trace-dfs                Stream the assembly search as JSON events\n"
        << "                             (\"[dfs] {...}\" lines) for the search viewer\n"
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
    std::vector<BackgroundEntry> background_entries;
    int    max_splits    = 2;
    double jig_clearance = 0.2;
    bool   trace_dfs     = false;
    std::string overrides_path, vacuum_model, gripper_model;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if      (arg == "--input"             && i + 1 < argc) { input_path        = argv[++i]; }
        else if (arg == "--output"            && i + 1 < argc) { output_path       = argv[++i]; }
        else if (arg == "--output-dir"        && i + 1 < argc) { output_dir        = argv[++i]; }
        else if (arg == "--slicer-config-dir" && i + 1 < argc) { slicer_config_dir = argv[++i]; }
        else if (arg == "--background-model"  && i + 1 < argc) {
            auto entry = parse_background_entry(argv[++i]);
            if (!entry) {
                std::cerr << "ERROR: invalid --background-model spec: " << argv[i] << "\n";
                return 1;
            }
            background_entries.push_back(std::move(*entry));
        }
        else if (arg == "--max-splits" && i + 1 < argc) { max_splits = std::atoi(argv[++i]); }
        else if (arg == "--jig-clearance" && i + 1 < argc) { jig_clearance = std::atof(argv[++i]); }
        else if (arg == "--trace-dfs") { trace_dfs = true; }
        else if (arg == "--grasp-overrides" && i + 1 < argc) { overrides_path = argv[++i]; }
        else if (arg == "--vacuum-model"    && i + 1 < argc) { vacuum_model  = argv[++i]; }
        else if (arg == "--gripper-model"   && i + 1 < argc) { gripper_model = argv[++i]; }
        else if (arg == "--no-grasps") { generate_grasps = false; }
        else if (arg == "--no-jigs")   { generate_jigs   = false; }
        else if (arg == "--no-path")   { generate_path   = false; }
        else if (arg == "--help" || arg == "-h") { print_usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << arg << "\n"; print_usage(argv[0]); return 1; }
    }

    if (input_path.empty()) { print_usage(argv[0]); return 1; }

    // Auto-load background models from config dir (if the file exists).
    // Entries from --background-model flags are appended after these.
    {
        const std::string bg_cfg = slicer_config_dir + "background_models.txt";
        std::ifstream bg_file(bg_cfg);
        if (bg_file) {
            std::string line;
            int line_no = 0;
            while (std::getline(bg_file, line)) {
                ++line_no;
                if (line.empty() || line[0] == '#') continue;
                auto entry = parse_background_entry(line);
                if (!entry) {
                    std::fprintf(stderr,
                        "[arms-plan] WARNING: skipping bad background_models.txt line %d: %s\n",
                        line_no, line.c_str());
                    continue;
                }
                background_entries.push_back(std::move(*entry));
            }
            if (!background_entries.empty())
                std::fprintf(stderr, "[arms-plan] Loaded %zu background model(s) from %s\n",
                             background_entries.size(), bg_cfg.c_str());
        }
    }

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
        assembler.setMaxSplits(max_splits);
        assembler.setCradleScalingDistance(static_cast<float>(jig_clearance));
        assembler.setTraceDfs(trace_dfs);

        // ---- hand-edited grasps ----
        if (!overrides_path.empty())
        {
            std::ifstream f(overrides_path);
            if (!f)
                throw std::runtime_error("cannot open grasp overrides: " + overrides_path);
            std::stringstream ss;
            ss << f.rdbuf();

            // Held in a named value: binding the range-for directly to
            // json::parse(...).as_array() would reference into a temporary that
            // dies before the first iteration.
            const json::value parsed = json::parse(ss.str());

            std::map<size_t, Assembler::GraspOverride> overrides;
            for (auto const& v : parsed.as_array())
            {
                auto const& o = v.as_object();
                Assembler::GraspOverride g;
                g.tool     = o.contains("tool") ? std::string(o.at("tool").as_string()) : "vacuum";
                g.dx_mm    = jnum_or(o, "dx_mm");
                g.dy_mm    = jnum_or(o, "dy_mm");
                g.dz_mm    = jnum_or(o, "dz_mm");
                g.width_mm = jnum_or(o, "width_mm");
                g.has_angle = o.contains("angle_rad");
                g.angle_rad = jnum_or(o, "angle_rad");
                overrides[static_cast<size_t>(jnum_or(o, "part_id"))] = g;
            }
            assembler.setGraspOverrides(overrides);
            std::fprintf(stderr, "[arms-plan] Applying %zu hand-edited grasp(s) from %s\n",
                         overrides.size(), overrides_path.c_str());
        }
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

        // ---- Background models ----
        // Load background STLs and embed as coloured GLBs.
        json::array bg_arr;
        std::vector<std::pair<std::string, std::vector<uint8_t>>> bg_glbs;  // name → bytes
        for (size_t bi = 0; bi < background_entries.size(); ++bi)
        {
            const auto& e = background_entries[bi];
            std::fprintf(stderr, "[arms-plan] Loading background: %s\n", e.path.c_str());
            MeshAsset bg_mesh = load_stl_world_space(e);
            if (bg_mesh.vertices.empty()) {
                std::fprintf(stderr, "[arms-plan] WARNING: background STL has no geometry: %s\n",
                             e.path.c_str());
                continue;
            }
            const std::string glb_name = "background_" + std::to_string(bi) + ".glb";
            bg_glbs.emplace_back(glb_name, mesh_to_glb_coloured(bg_mesh, e.r, e.g, e.b));

            json::object bg_entry;
            bg_entry["file"] = glb_name;
            bg_entry["r"]    = static_cast<double>(e.r);
            bg_entry["g"]    = static_cast<double>(e.g);
            bg_entry["b"]    = static_cast<double>(e.b);
            bg_arr.push_back(std::move(bg_entry));
        }
        manifest["background"] = std::move(bg_arr);

        // ---- Parallel-plate grasps ----
        // The grasps array above covers the vacuum cup only.  Parts picked with
        // the gripper get a record of where its two jaws sit, in the same world-mm
        // convention, so the viewer can draw them and re-anchor them per stage.
        {
            const auto& gc = assembler.gripperConfig();
            json::array ppg_arr;
            for (auto const& [part, assembled_pos] : target_assembly->getAssembledPartTransforms())
            {
                if (part->getGraspTool() != Part::GraspTool::PPG) continue;
                const PPGGrasp& g = part->getPPGGrasp();

                json::object e;
                e["part_id"]          = static_cast<int64_t>(part->getId());
                e["x_mm"]             = assembled_pos.X() + g.position_.X();
                e["y_mm"]             = assembled_pos.Y() + g.position_.Y();
                e["z_mm"]             = assembled_pos.Z() + g.position_.Z();
                e["jaw_bottom_z_mm"]  = assembled_pos.Z() + g.jaw_z_;
                e["angle_rad"]        = g.rotation_;
                e["width_mm"]         = g.width_;
                e["jaw_width_mm"]     = gc.jaw_width_mm;
                e["jaw_height_mm"]    = gc.jaw_height_mm;
                e["jaw_thickness_mm"] = gc.jaw_thickness_mm;
                ppg_arr.push_back(std::move(e));
            }
            if (!ppg_arr.empty())
                std::fprintf(stderr, "[arms-plan] %zu parallel-plate grasp(s)\n", ppg_arr.size());
            manifest["ppg_grasps"] = std::move(ppg_arr);
        }

        // ---- Tool models ----
        // Baked in like the nozzle so the viewer draws the real hardware.  These
        // are visual only: collision still uses the simple nozzle primitive.
        std::vector<std::pair<std::string, std::vector<uint8_t>>> tool_glbs;
        json::object tools_obj;

        auto bake = [&](const std::string& path, const char* key, bool split) {
            if (path.empty() || !fs::exists(path)) return;
            ToolMesh body, jlo, jhi;
            if (!bake_tool(path, split, body, jlo, jhi)) {
                std::fprintf(stderr, "[arms-plan] WARNING: could not read tool model %s\n",
                             path.c_str());
                return;
            }
            json::object t;
            const std::string stem = std::string("tool_") + key;
            tool_glbs.emplace_back(stem + ".glb", mesh_to_glb(body.mesh));
            t["body_file"] = stem + ".glb";
            if (!jlo.mesh.vertices.empty() && !jhi.mesh.vertices.empty()) {
                tool_glbs.emplace_back(stem + "_jaw_lo.glb", mesh_to_glb(jlo.mesh));
                tool_glbs.emplace_back(stem + "_jaw_hi.glb", mesh_to_glb(jhi.mesh));
                t["jaw_lo_file"]    = stem + "_jaw_lo.glb";
                t["jaw_hi_file"]    = stem + "_jaw_hi.glb";
                t["jaw_lo_inner_mm"]  = jlo.inner_mm;
                t["jaw_hi_inner_mm"]  = jhi.inner_mm;
                t["jaw_bottom_dz_mm"] = jlo.jaw_z0_mm;

                // The arms, so the viewer can swing the whole mechanism together
                // rather than sliding the jaws in isolation.
                json::array arms;
                auto emit_arms = [&](const ToolMesh& jm, int sign) {
                    int k = 0;
                    for (auto const& l : jm.links) {
                        const std::string f = stem + "_arm" +
                            std::string(sign < 0 ? "_lo_" : "_hi_") + std::to_string(k++) + ".glb";
                        tool_glbs.emplace_back(f, mesh_to_glb(l.mesh));
                        json::object a;
                        a["file"]       = f;
                        a["sign"]       = sign;
                        a["pivot_x_mm"] = l.body.x;
                        a["pivot_z_mm"] = l.body.z;
                        a["length_mm"]  = l.length();
                        arms.push_back(std::move(a));
                    }
                };
                emit_arms(jlo, -1);
                emit_arms(jhi, +1);
                if (!arms.empty()) {
                    t["arms"] = std::move(arms);
                    // Half-opening when the arms sit at their modelled angle;
                    // the viewer solves back from this for any other opening.
                    t["arm_length_mm"] = jlo.links.empty() ? 0.0 : jlo.links.front().length();
                }
            }
            tools_obj[key] = std::move(t);
            std::fprintf(stderr, "[arms-plan] Tool model '%s' baked from %s\n", key, path.c_str());
        };

        bake(vacuum_model,  "vacuum",  false);
        // The cup's contact face sits NOZZLE_H_HALF below the grasp point the
        // planner records (see VacuumGraspGenerator), and the model's origin is
        // its tip — so this is how far to drop it.
        if (tools_obj.contains("vacuum"))
            tools_obj["vacuum"].as_object()["origin_dz_mm"] = -10.0;
        bake(gripper_model, "gripper", true);
        manifest["tools"] = std::move(tools_obj);

        // ---- Vacuum nozzle ----
        // Bake the tool the planner actually reasoned with into the .arms, so the
        // viewer draws that geometry rather than guessing at a shape.  The mesh is
        // local-frame metres with the contact face at z = -0.01 m and any tool XYZ
        // correction already applied, so the viewer can place it directly at a
        // grasp's (x_mm, y_mm, z_mm) — that point IS the nozzle centroid.
        std::vector<uint8_t> nozzle_glb;
        if (auto nozzle_mesh = assembler.getNozzleMesh())
        {
            nozzle_glb = mesh_to_glb(*nozzle_mesh);
            manifest["nozzle_file"] = "nozzle.glb";
            std::fprintf(stderr, "[arms-plan] Nozzle mesh: %zu verts, %zu tris\n",
                         nozzle_mesh->vertices.size(), nozzle_mesh->triangles.size());
        }

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

        // Add background GLBs
        for (auto const& [name, bytes] : bg_glbs)
            zip.add(name, bytes);

        if (!nozzle_glb.empty())
            zip.add("nozzle.glb", nozzle_glb);

        for (auto const& [name, bytes] : tool_glbs)
            zip.add(name, bytes);

        std::fprintf(stderr, "[arms-plan] Done. %zu parts, %zu stages, %zu grasps, %zu GLBs, "
                     "%zu background meshes → %s\n",
                     target_assembly->getAssembledPartTransforms().size(),
                     path.size(), grasps.size(), glb_count, bg_glbs.size(),
                     output_path.c_str());
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "[arms-plan] FATAL: %s\n", e.what());
        return 1;
    }

    return 0;
}
