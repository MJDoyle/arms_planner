#ifndef ASSEMBLER_HPP
#define ASSEMBLER_HPP

#include "assembler/Assembly.hpp"
#include "assembler/SceneModel.hpp"
#include "assembler/CollisionAdapter.hpp"
#include "assembler/MeshAsset.hpp"
#include "assembler/VacuumGraspGenerator.hpp"

#include <string>
#include <memory>
#include <optional>
#include <vector>
#include <map>
#include <set>


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

class Assembler {

public:
    explicit Assembler();

    void generateAssemblySequence();

    void setTargetAssembly(std::shared_ptr<Assembly> target_assembly) { target_assembly_ = target_assembly; }

    std::shared_ptr<Assembly> getInitialAssembly() { return initial_assembly_; }
    std::shared_ptr<Assembly> getTargetAssembly() { return target_assembly_; }
    std::vector<std::shared_ptr<AssemblyNode>> getAssemblyPath() { return assembly_path_; }
    std::string getName() { return name_; }
    std::shared_ptr<MeshAsset> getNozzleMesh() const { return nozzle_mesh_; }

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
    void setToolConfig(const ToolConfig& cfg) { tool_config_ = cfg; }

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

    // Check whether removing `part` from `assembled` is physically feasible.
    // Returns nullopt if infeasible; otherwise the grasp position in local frame
    // (relative to part centroid, mm) — zero for parts without a grasp (internal).
    // Checks performed:
    //   - Coal z-step lift (all parts)
    //   - VacuumGraspGenerator with Coal assembly check (external parts)
    //   - Gripper-vs-jig pick check (external parts with bay assigned)
    std::optional<gp_Pnt> edge_feasible(
        std::shared_ptr<Part> part,
        const PartTransformMap& assembled);

    std::vector<std::shared_ptr<AssemblyNode>> breadthFirstZAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> depthFirstZAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> findNodeNeighbours(std::shared_ptr<AssemblyNode> node);

    // Neutral scene model and Coal backend.  Built in generateInitialAssembly.
    SceneModel scene_;
    std::unique_ptr<CollisionAdapter> collision_adapter_;

    // Tessellated vacuum nozzle mesh (shared across all edge checks).
    std::shared_ptr<MeshAsset> nozzle_mesh_;

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
    ToolConfig tool_config_;
    std::string run_output_dir_;
    std::string slicer_config_dir_;

};

#endif  // ASSEMBLER_HPP