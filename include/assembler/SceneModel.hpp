#pragma once

#include "assembler/MeshAsset.hpp"

#include <gp_Trsf.hxx>

#include <map>
#include <memory>
#include <string>
#include <vector>

enum class SceneRole { Part, Gripper, Jig, Fixture };

struct SceneObject {
    std::shared_ptr<MeshAsset> mesh;
    SceneRole role{SceneRole::Part};
    // Pose: translation in metres, rotation dimensionless.  Applied to the
    // local-frame mesh (bbox centroid at origin) to yield world coordinates.
    gp_Trsf pose;
    bool present{true};
};

// Neutral scene model.  The authoritative scene state: a collection of named
// objects each with geometry, a semantic role, a world pose, and a present/absent
// flag.  No collision-library types appear here; the CoalAdapter is a consumer.
class SceneModel {
public:
    void add_object(const std::string& id,
                    std::shared_ptr<MeshAsset> mesh,
                    SceneRole role,
                    const gp_Trsf& pose,
                    bool present = true);

    void remove_object(const std::string& id);

    // Toggle whether an object participates in collision queries without
    // removing its geometry (used to mark parts as assembled / removed).
    void set_present(const std::string& id, bool present);

    void set_pose(const std::string& id, const gp_Trsf& pose);

    bool has_object(const std::string& id) const;

    const SceneObject& get_object(const std::string& id) const;

    // IDs of all currently present objects.
    std::vector<std::string> present_object_ids() const;

    // Read-only access to the full map (used by CoalAdapter::sync).
    const std::map<std::string, SceneObject>& objects() const { return objects_; }

    void clear() { objects_.clear(); }

private:
    std::map<std::string, SceneObject> objects_;
};
