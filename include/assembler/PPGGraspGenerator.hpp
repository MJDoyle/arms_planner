#ifndef PPG_GRASP_GENERATOR_HPP
#define PPG_GRASP_GENERATOR_HPP

#include "assembler/CollisionAdapter.hpp"
#include "assembler/MeshFunctions.hpp"

#include <optional>
#include <string>
#include <vector>

struct GripperConfig;

// A parallel-plate grasp.  The jaws close along a horizontal axis and the tool
// rotates about z only, so a grasp is fully described by where the jaws meet the
// part, the axis angle, and how far apart they sit.
struct PPGGrasp
{
    gp_Vec        position_;          // grasp centre relative to the part centroid (mm)
    Standard_Real rotation_ = 0.0;    // jaw axis angle about +z (rad)
    Standard_Real width_    = 0.0;    // jaw opening at contact (mm)
    Standard_Real jaw_z_    = 0.0;    // underside of the jaws, relative to centroid (mm)
    bool          valid_    = false;
};

// A descriptive label for the contact, used in reporting.  Ranking is done by the
// continuous score below, not by this: the two properties trade off against each
// other, so a small contact on a vertical edge can beat a broad one on a slope.
enum class PPGContactClass {
    VerticalFace   = 0,   // broad and vertical
    VerticalLine   = 1,   // vertical but narrow across the jaw (e.g. a cylinder)
    SlopedFace     = 2,   // broad but tilted
    PointContact   = 3,   // narrow in both directions
    Infeasible     = 4
};

// One evaluated jaw placement, before the assembled-parts check.
struct PPGCandidate
{
    double          angle_rad  = 0.0;   // jaw axis about +z
    double          jaw_z_mm   = 0.0;   // underside of the jaws, world mm
    double          width_mm   = 0.0;   // opening at contact
    gp_Pnt          centre;             // midpoint between the two contacts, world mm
    PPGContactClass klass      = PPGContactClass::Infeasible;
    double          moment_arm = 0.0;   // perpendicular distance from CoM to the grasp axis
    double          quality    = 0.0;   // 0..1, higher is better — see PPGGraspGenerator.cpp
    double          slope_rad  = 0.0;   // worst of the two contacts
    double          contact_mm2 = 0.0;  // smaller of the two contact patches
};

// Everything about a part's parallel-plate grasps that does not depend on which
// other parts are present — the direct analogue of VacuumGraspGenerator's
// precomputed candidate set.
struct PartPPGCandidates
{
    std::vector<PPGCandidate> candidates;   // best-first
};

class Part;

class PPGGraspGenerator
{
public:
    // Geometry-only pass.  Sweeps jaw angle and height, finds where the jaws would
    // touch, classifies the contact, and keeps everything inside the friction cone.
    // Independent of the assembly state, so it runs once per part.
    static PartPPGCandidates precompute(std::shared_ptr<Part> part,
                                        const GripperConfig&  cfg);

    // Pick the best cached candidate that also clears `assembled_ids`, and whose
    // jaws can descend to it without striking anything.
    static std::optional<PPGGrasp> select(
        std::shared_ptr<Part>            part,
        const PartPPGCandidates&         candidates,
        const GripperConfig&             cfg,
        CollisionAdapter&                adapter,
        const std::vector<std::string>&  assembled_ids);

    // Solid swept by one jaw, in world coordinates.  `side` is +1 or -1 along the
    // grasp axis.  With `sweep_to` above the jaw, this is the descent volume rather
    // than the jaw itself — used both for approach checks and for cutting jig slots.
    static TopoDS_Shape jawSolid(const PPGCandidate&  cand,
                                 const GripperConfig& cfg,
                                 int                  side,
                                 double               sweep_to_z = 0.0);
};

#endif
