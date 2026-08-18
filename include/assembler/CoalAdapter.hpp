#pragma once

#include "assembler/CollisionAdapter.hpp"

// Coal types are confined to this header and CoalAdapter.cpp.
// Nothing outside the assembler/CoalAdapter translation unit should include
// coal headers directly.
#include <coal/BVH/BVH_model.h>
#include <coal/collision_object.h>

#include <map>
#include <memory>
#include <string>

class CoalAdapter : public CollisionAdapter {
public:
    // safety_margin_m: minimum clearance (m) required to report collision-free.
    // Derive from tessellation deflection: deflection_mm * 0.5 * 0.001.
    explicit CoalAdapter(double safety_margin_m);

    // Rebuild Coal collision objects from all present scene objects.
    // Geometry (BVH) is cached per MeshAsset pointer; only the transform is
    // updated when the same mesh reappears with a new pose.
    void sync(const SceneModel& scene) override;

    void add_or_update(const std::string& id,
                       std::shared_ptr<MeshAsset> mesh,
                       const gp_Trsf& pose) override;

    void remove(const std::string& id) override;

    bool collision_free(const std::string& id_a,
                        const std::string& id_b) const override;

    double min_distance(const std::string& id_a,
                        const std::string& id_b) const override;

private:
    coal::Transform3s to_coal_transform(const gp_Trsf& trsf) const;

    // Build BVH from a MeshAsset; result is cached in geom_cache_.
    std::shared_ptr<coal::CollisionGeometry>
    get_or_build_geometry(const MeshAsset& mesh);

    double safety_margin_m_;

    // id → coal collision object (present objects only, rebuilt on sync)
    std::map<std::string, std::shared_ptr<coal::CollisionObject>> objects_;

    // MeshAsset raw pointer → coal geometry (avoids rebuilding BVH for unchanged meshes)
    std::map<const MeshAsset*, std::shared_ptr<coal::CollisionGeometry>> geom_cache_;
};
