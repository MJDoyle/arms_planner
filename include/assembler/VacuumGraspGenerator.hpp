#ifndef VACUUM_GRASP_GENERATOR_HPP
#define VACUUM_GRASP_GENERATOR_HPP

#include "assembler/CollisionAdapter.hpp"
#include "assembler/MeshAsset.hpp"
#include "assembler/MeshFunctions.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

class Part;

struct GraspAttempt {
    double x_mm, y_mm, z_mm;  // nozzle centroid position in world frame (mm)
    enum class Status { body_collision, assembly_collision, seal_failed, accepted } status;
};

class VacuumGraspGenerator
{
public:
    // Find the best top-down vacuum grasp for `part` that is clear of both
    // the part body and every object listed in `assembled_ids`.
    //
    // `adapter`      — collision backend (part must already be present in it)
    // `nozzle_mesh`  — tessellated nozzle mesh (local frame, m)
    // `assembled_ids`— scene IDs of the other assembled parts to avoid
    // `debug_out`    — if non-null, every attempted position + rejection reason
    //                  is appended (world frame, mm)
    //
    // Returns the grasp position in local frame (relative to shape centroid, mm)
    // — i.e. the nozzle contact point offset from the centroid — or nullopt if no
    // valid grasp exists.
    static std::optional<gp_Pnt> generate(
        std::shared_ptr<Part>              part,
        CollisionAdapter&                  adapter,
        const std::shared_ptr<MeshAsset>&  nozzle_mesh,
        const std::vector<std::string>&    assembled_ids,
        std::vector<GraspAttempt>*         debug_out = nullptr);
};

#endif
