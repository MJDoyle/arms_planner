#ifndef VACUUM_GRASP_GENERATOR_HPP
#define VACUUM_GRASP_GENERATOR_HPP

#include "assembler/CollisionAdapter.hpp"
#include "assembler/MeshFunctions.hpp"

#include <TopoDS_Shape.hxx>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class Part;

struct GraspAttempt {
    size_t part_id;            // owning part's Part::getId() — lets the viewer
                                // re-anchor this marker to the part's current
                                // (assembled or bay) position instead of the
                                // fixed assembled-frame position it was computed at
    double x_mm, y_mm, z_mm;  // nozzle centroid position in world frame (mm)
    enum class Status { body_collision, assembly_collision, seal_failed, accepted } status;
    // True on the single accepted attempt that generate() actually returned for
    // this part — the one closest to the centre of mass.  Every other accepted
    // attempt is a viable alternative that was not picked.
    bool chosen = false;
};

// One nozzle placement that has already been verified clear of the part's own
// body and passes the seal check — i.e. everything about a candidate that does
// NOT depend on which other parts happen to be assembled.  world_x_mm/world_y_mm
// plus nozzle_cen_z_mm are enough to re-test the position against a given
// assembled-parts set without repeating face/seal geometry work.
struct GraspCandidate {
    double world_x_mm, world_y_mm, nozzle_cen_z_mm;
    gp_Pnt local_pos;    // grasp result in local frame (mm), as returned by generate()
    double com_dist_mm;  // distance from candidate to the part's centre of mass (mm)
};

// Geometrically-valid candidates for one part, grouped by source face, each
// list in the same spiral-search order VacuumGraspGenerator uses internally.
// A placement rejected by the geometry-only pass, kept so the viewer can show
// why without anyone re-running the search.
struct RejectedCandidate {
    double world_x_mm, world_y_mm, nozzle_cen_z_mm;
    GraspAttempt::Status reason;   // body_collision or seal_failed
};

struct PartGraspCandidates {
    std::vector<std::vector<GraspCandidate>> faces;
    // Placements that failed the body or seal check, in search order.
    std::vector<RejectedCandidate> rejected;
};

class VacuumGraspGenerator
{
public:
    // Find the best top-down vacuum grasp for `part` that is clear of both
    // the part body and every object listed in `assembled_ids`.
    //
    // `adapter`       — collision backend (part must already be present in it)
    // `nozzle_shape`  — nozzle geometry in local-frame metres (bbox centroid at origin)
    // `assembled_ids` — scene IDs of the other assembled parts to avoid
    // `debug_out`     — if non-null, every attempted position + rejection reason
    //                   is appended (world frame, mm)
    //
    // Returns the grasp position in local frame (relative to shape centroid, mm)
    // — i.e. the nozzle contact point offset from the centroid — or nullopt if no
    // valid grasp exists.
    static std::optional<gp_Pnt> generate(
        std::shared_ptr<Part>                part,
        CollisionAdapter&                    adapter,
        const std::shared_ptr<TopoDS_Shape>& nozzle_shape,
        const std::vector<std::string>&      assembled_ids,
        std::vector<GraspAttempt>*           debug_out = nullptr);

    // Geometry-only pass: enumerate every nozzle placement on `part` that
    // clears the part's own body and passes the seal check — independent of
    // which other parts are assembled, so the result can be reused across
    // every DFS node that re-considers this part, instead of re-deriving it
    // each time.  `adapter` should be a private instance used only for this
    // call (this function registers `part` into it) — safe to run
    // concurrently across parts, each with its own adapter instance.
    //
    // `local_frame_shape` and `nozzle_shape` must already be tessellated
    // (BRepMesh_IncrementalMesh already run on them) before calling this
    // concurrently — OCCT tessellation mutates the shape's internal
    // triangulation cache, so triangulating on first use here would race
    // if two threads' shapes happen to share underlying geometry (the
    // nozzle_shape is literally the same object across every call).
    static PartGraspCandidates precompute(
        std::shared_ptr<Part>                part,
        const gp_Pnt&                        world_pos_mm,
        const std::shared_ptr<TopoDS_Shape>& local_frame_shape,
        CollisionAdapter&                    adapter,
        const std::shared_ptr<TopoDS_Shape>& nozzle_shape);

    // Fast pass: given candidates already known to be clear of the part body
    // and sealed, pick the best one that's also clear of `assembled_ids`.
    // Equivalent to generate() but without repeating the face/seal work.
    // Pass debug_out to also record every attempt behind the decision — the
    // geometry-only rejects cached by precompute(), the candidates knocked out
    // by the assembled parts here, and the winner (flagged `chosen`).  That is
    // the same record generate(debug_out) produces, without a second search, so
    // the visualisation is guaranteed to describe the grasp actually selected.
    static std::optional<gp_Pnt> select(
        std::shared_ptr<Part>                part,
        const PartGraspCandidates&           candidates,
        CollisionAdapter&                    adapter,
        const std::shared_ptr<TopoDS_Shape>& nozzle_shape,
        const std::vector<std::string>&      assembled_ids,
        std::vector<GraspAttempt>*           debug_out = nullptr);
};

#endif
