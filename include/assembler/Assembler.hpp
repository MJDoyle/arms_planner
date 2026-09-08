#ifndef ASSEMBLER_HPP
#define ASSEMBLER_HPP

#include "assembler/Assembly.hpp"
#include "assembler/SceneModel.hpp"
#include "assembler/CollisionAdapter.hpp"
#include "assembler/MeshAsset.hpp"
#include "assembler/VacuumGraspGenerator.hpp"
#include "assembler/PPGGraspGenerator.hpp"

#include <string>
#include <memory>
#include <optional>
#include <vector>
#include <map>
#include <set>
#include <functional>


class aiScene;

// Describes a vacuum tool: which mesh file to load and an XYZ correction
// (in mm) applied to mesh vertices after tessellation.  The correction shifts
// the mesh so the contact face sits at the correct position relative to the
// part.  Z is usually left at 0 — the loader already aligns the contact face
// automatically; X/Y correct any lateral offset visible in the visualisation.
struct ToolConfig {
    std::string name;
    std::string mesh_file;       // absolute path to STEP/OBJ/STL
    double offset_x_mm = 0.0;
    double offset_y_mm = 0.0;
    double offset_z_mm = 0.0;
};

// Parallel-plate gripper, used when the vacuum cup cannot seal.  The jaws close
// along a horizontal axis and the whole tool rotates about z, so a grasp is
// (position, angle, opening) — see PPGGraspGenerator.
struct GripperConfig {
    double jaw_width_mm     = 10.0;  // along the jaw face, horizontal
    double jaw_height_mm    = 12.0;  // along the jaw face, vertical
    double jaw_thickness_mm =  4.0;  // through the jaw
    double min_opening_mm   =  2.0;
    double max_opening_mm   = 60.0;
    double friction_mu      =  0.5;  // jaw pad against part; sets the max face slope
    // Extra clearance cut either side of the jaws when slotting a jig.
    double slot_clearance_mm = 1.0;
};

class Assembler {

public:
    explicit Assembler();

    void generateAssemblySequence();

    void setTargetAssembly(std::shared_ptr<Assembly> target_assembly) { target_assembly_ = target_assembly; }

    std::shared_ptr<Assembly> getInitialAssembly() { return initial_assembly_; }
    std::shared_ptr<Assembly> getTargetAssembly() { return target_assembly_; }
    std::vector<std::shared_ptr<AssemblyNode>> getAssemblyPath() { return assembly_path_; }
    std::string getName() { return name_; }
    std::shared_ptr<MeshAsset>    getNozzleMesh()  const { return nozzle_mesh_;  }
    std::shared_ptr<TopoDS_Shape> getNozzleShape() const { return nozzle_shape_; }

    void setName(std::string name) { name_ = name; }

    // Directory for all outputs of this pipeline run (e.g. "assembler_output/test_blocks/").
    // Must end with '/'.  Created by the node before calling generateAssemblySequence().
    void setRunOutputDir(const std::string& dir) { run_output_dir_ = dir; }
    const std::string& getRunOutputDir() const   { return run_output_dir_; }

    // Directory containing arms_prusa_config.ini.  Must end with '/'.
    // Defaults to the WORKING_DIR constant ("assembler_working/").
    void setSlicerConfigDir(const std::string& dir) { slicer_config_dir_ = dir; }
    const std::string& getSlicerConfigDir() const   { return slicer_config_dir_; }

    void setGenerateGrasps(bool v) { generate_grasps_ = v; }
    void setGenerateJigs(bool v) { generate_jigs_ = v; }
    void setGeneratePath(bool v) { generate_path_ = v; }
    void setCollisionVolumeThreshold(double v) { collision_volume_threshold_ = v; }
    void setCradleScalingDistance(float v) { cradle_scaling_distance_ = v; }

    // 0 disables splitting entirely.
    void setMaxSplits(int v) { max_splits_ = v; }

    // Stream the search as it runs, as one JSON object per line prefixed "[dfs] ".
    // Off by default: collecting the rejection reasons costs extra collision
    // queries, and it is only wanted when someone is watching.
    void setTraceDfs(bool v) { trace_dfs_ = v; }

    // A grasp edited by hand in the viewer, replacing whatever the planner would
    // have chosen for that part.  Offsets are in the part's own frame, which is
    // why one value covers both the pick and the place.
    struct GraspOverride {
        std::string tool;        // "vacuum" | "gripper"
        double dx_mm = 0.0, dy_mm = 0.0, dz_mm = 0.0;
        double width_mm = 0.0;   // gripper only; 0 keeps the planner's opening
        // Zero is a valid jaw angle, so presence is tracked separately.
        double angle_rad = 0.0;
        bool   has_angle = false;
    };
    void setGraspOverrides(const std::map<size_t, GraspOverride>& o) { grasp_overrides_ = o; }
    void setToolConfig(const ToolConfig& cfg) { tool_config_ = cfg; }
    void setGripperConfig(const GripperConfig& cfg) { gripper_config_ = cfg; }
    const GripperConfig& gripperConfig() const { return gripper_config_; }

    void reset();  // Clear all run-specific state so a new model can be loaded cleanly

    // Run VacuumGraspGenerator on every external part with the full assembly as
    // collision context, collecting every attempted nozzle position and its
    // rejection reason.  Call after generateAssemblySequence() for debug viz.
    std::vector<GraspAttempt> debugGrasps();

private:

    void saveAssemblyPath();

    bool loadAssemblyPath();

    void initialisePartBays();

    void generateInitialAssembly();

    void generateInitialPartPositions();

    void generateGCodeFile(std::vector<size_t> part_addition_order);

    void generateCommandFile(std::vector<size_t> part_addition_order);

    std::vector<size_t> generatePartAdditionOrder();

    size_t nodeIdGenerator(std::vector<size_t> object_ids);

    std::map<size_t, std::vector<size_t>> node_id_map_;

    size_t next_node_ID_ = 0;

    void generateNegatives();

    void generateSlicerGcode();

    void generateGrasps();

    void debugState();

    bool arrangeInternalParts();

    //void alignTargetAssemblyToInitialAssembly();

    void alignAssemblyPathToInitialAssembly();

    // Pre-assign bay indices and positions for all external parts before DFS.
    void assignExternalBayPositions();

    // Compute geometry-only grasp candidates (VacuumGraspGenerator::precompute)
    // for every external part once, in parallel, before the DFS starts.
    // edge_feasible() then only re-runs the cheap assembled-parts check
    // against these cached candidates instead of the full search.
    void precomputeGraspCandidates();

    // The (part, assembled-part-IDs) pairs at which each external part should be
    // grasp-checked.  See Assembler.cpp for why the assembly path matters here.
    std::vector<std::pair<std::shared_ptr<Part>, std::vector<std::string>>>
    graspEvaluationContexts() const;

    // Re-point the collision scene at the target assembly's current transforms.
    // Must be called before any collision query that runs after
    // alignAssemblyPathToInitialAssembly() has moved the assembly.
    void refreshCollisionScene();

    void applyGraspOverride(const std::shared_ptr<Part>& part,
                            const GraspOverride& ov,
                            const std::vector<std::string>& assembled_ids);

    // Check whether removing `part` from `assembled` is physically feasible.
    // Returns nullopt if infeasible; otherwise the grasp position in local frame
    // (relative to part centroid, mm) — zero for parts without a grasp (internal).
    // Checks performed:
    //   - Coal z-step lift (all parts)
    //   - VacuumGraspGenerator with Coal assembly check (external parts)
    //   - Gripper-vs-jig pick check (external parts with bay assigned)
    // Successors reachable only by splitting a printed part.  Consulted solely
    // when findNodeNeighbours() comes back empty — see Assembler.cpp.
    std::vector<std::shared_ptr<AssemblyNode>> findSplitNeighbours(
        std::shared_ptr<AssemblyNode> node);

    // Register a freshly cut piece in the scene and collision backend so the
    // feasibility checks and later DFS nodes can see it.
    std::shared_ptr<Part> registerSplitPiece(const TopoDS_Shape& shape,
                                             const std::string&  name);

    // Undo registerSplitPiece for a candidate that did not work out.  Without
    // this the scene keeps every piece the search ever speculatively cut.
    void unregisterSplitPiece(const std::shared_ptr<Part>& piece);

    // Warn when a cut leaves a large unsupported first layer.  Advisory only:
    // support is the model designer's call, this just flags it.
    void warnIfUnsupported(const TopoDS_Shape& upper,
                           const TopoDS_Shape& lower,
                           const TopoDS_Shape& placed_part,
                           double z_cut,
                           const std::string& name) const;

    std::optional<gp_Pnt> edge_feasible(
        std::shared_ptr<Part> part,
        const PartTransformMap& assembled,
        std::vector<std::shared_ptr<Part>>* blockers = nullptr);

    std::vector<std::shared_ptr<AssemblyNode>> breadthFirstZAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> depthFirstZAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> findNodeNeighbours(std::shared_ptr<AssemblyNode> node);

    // Neutral scene model and collision backend.  Built in generateInitialAssembly.
    SceneModel scene_;
    std::unique_ptr<CollisionAdapter> collision_adapter_;

    // Constructs a fresh CollisionAdapter of whatever concrete type
    // collision_adapter_ is — used by precomputeGraspCandidates() to give
    // each worker thread its own private, unshared adapter instance.
    std::function<std::unique_ptr<CollisionAdapter>()> make_collision_adapter_;

    // Geometry-only grasp candidates per external part (by Part ID), built
    // once by precomputeGraspCandidates() before the DFS starts.
    std::map<size_t, PartGraspCandidates> grasp_candidates_;

    // Parallel-plate candidates, same lifecycle as the vacuum ones: geometry-only,
    // computed once per external part before the search.
    std::map<size_t, PartPPGCandidates> ppg_candidates_;

    // Every grasp attempt behind the selected grasps, recorded by the single
    // authoritative pass in generateGrasps().  debugGrasps() just hands this to
    // the visualiser — nothing re-runs the search, so what is drawn always
    // describes the grasp the machine will use.
    std::vector<GraspAttempt> grasp_attempts_;

    // Vacuum nozzle geometry used across all edge checks.
    // nozzle_mesh_ — tessellated for visualisation; nozzle_shape_ — B-rep for collision.
    // Both are in local-frame metres with the bbox centroid at the origin.
    std::shared_ptr<MeshAsset>    nozzle_mesh_;
    std::shared_ptr<TopoDS_Shape> nozzle_shape_;

    std::shared_ptr<Assembly> initial_assembly_;

    std::shared_ptr<Assembly> target_assembly_;

    std::vector<std::shared_ptr<AssemblyNode>> assembly_path_;

    std::shared_ptr<Part> base_part_;

    std::string output_path_;
    std::string input_path_;

    std::vector<std::string> slicer_gcode_;

    std::vector<std::vector<bool>> bay_occupancy_;

    std::string name_;

    bool generate_grasps_ = true;
    bool generate_jigs_ = true;
    bool generate_path_ = true;
    double collision_volume_threshold_ = 0.0;
    float cradle_scaling_distance_ = 0.2f;

    // Maximum number of printed-part splits the search may use.  Splits are a
    // fallback for dead-end states; a piece produced by one may itself be split
    // again, so this caps the total across the whole search.
    // Splits taken on the chosen path.  The source part is still printed as one
    // object; z_cut_rel is where its g-code is divided so the freed part can be
    // dropped in mid-print.
    struct SplitRecord {
        std::shared_ptr<Part> source, lower, upper, freed;
        double z_cut_rel = 0.0;   // mm above the part's own underside
    };
    std::vector<SplitRecord> splits_;

    // Hand edits, applied in generateGrasps() so the command file and g-code
    // carry them.  The sequence itself is left alone: the user changed how a part
    // is picked, not the order it goes in.
    std::map<size_t, GraspOverride> grasp_overrides_;

    // Local-frame collision shape per part ID, built on first use and kept alive
    // for the run.  Also keeps the adapter's geometry cache keys stable.
    std::map<size_t, std::shared_ptr<TopoDS_Shape>> lf_shape_cache_;

    // Slicer output divided at each split height; one entry when nothing is split.
    std::vector<std::vector<std::string>> slicer_gcode_segments_;

    static std::vector<std::vector<std::string>> splitGcodeAtHeights(
        const std::vector<std::string>& gcode, std::vector<double> heights);

    bool isSplitPiece(const std::shared_ptr<Part>& p) const {
        for (auto const& s : splits_)
            if (p == s.lower || p == s.upper) return true;
        return false;
    }

    bool   trace_dfs_    = false;
    void   traceDfs(const std::string& json) const;

    int    max_splits_   = 2;
    int    splits_used_  = 0;
    // Cutting is tried at every dead end and each attempt costs boolean geometry,
    // so the work is capped as well as the number of cuts actually taken.
    int    split_attempts_ = 0;
    size_t next_split_id_ = 0;
    ToolConfig    tool_config_;
    GripperConfig gripper_config_;
    std::string run_output_dir_;
    std::string slicer_config_dir_;

};

#endif  // ASSEMBLER_HPP