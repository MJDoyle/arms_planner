#pragma once

#include "assembler/SceneModel.hpp"

#include <string>

// Abstract collision backend.  No coal/fcl types appear here or in callers.
// All scene state flows in through sync(); query methods are pure reads.
class CollisionAdapter {
public:
    virtual ~CollisionAdapter() = default;

    // Rebuild internal collision objects to match the current scene.
    // Call after any add_object / set_present / set_pose mutation.
    virtual void sync(const SceneModel& scene) = 0;

    // Add or update a single collision object without a full re-sync.
    // Used for temporary objects (gripper, jig) during edge feasibility checks.
    virtual void add_or_update(const std::string& id,
                                std::shared_ptr<MeshAsset> mesh,
                                const gp_Trsf& pose) = 0;

    // Remove a single collision object.
    virtual void remove(const std::string& id) = 0;

    // True iff the two named objects (both must be present) are not in contact,
    // accounting for the safety margin derived from the tessellation deflection.
    // Returns true if either object is absent.
    virtual bool collision_free(const std::string& id_a,
                                const std::string& id_b) const = 0;

    // Signed minimum distance (m) between the two objects.  Negative means
    // penetration depth.  Returns +inf if either object is absent.
    virtual double min_distance(const std::string& id_a,
                                const std::string& id_b) const = 0;
};
