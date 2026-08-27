#pragma once

#include "assembler/CollisionAdapter.hpp"

// Coal types are confined to this header and CoalAdapter.cpp.
// Nothing outside the assembler/CoalAdapter translation unit should include
// coal headers directly.
#include <coal/BVH/BVH_model.h>
#include <coal/collision_object.h>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

class CoalAdapter : public CollisionAdapter {
public:
    // safety_margin_m: minimum clearance (m) required to report collision-free.
    // Derive from tessellation deflection: deflection_mm * 0.5 * 0.001.
    explicit CoalAdapter(double safety_margin_m);

    // Rebuild Coal collision objects from all present scene objects.
    // Geometry (BVH) is built from each object's already-tessellated
    // MeshAsset (cheap — no re-tessellation) and cached per MeshAsset pointer.
    void sync(const SceneModel& scene) override;

    // Single-object add/update (transient objects — nozzle, lifted-part
    // copies — that aren't in the SceneModel).  `shape` is tessellated on
    // first use and the BVH is cached per TopoDS_Shape pointer, so repeated
    // calls with the same shared_ptr (e.g. the nozzle, re-posed every
    // candidate) only pay the tessellation cost once.
    void add_or_update(const std::string& id,
                       std::shared_ptr<TopoDS_Shape> shape,
                       const gp_Trsf& pose) override;

    void remove(const std::string& id) override;

    bool collision_free(const std::string& id_a,
                        const std::string& id_b) const override;

    double min_distance(const std::string& id_a,
                        const std::string& id_b) const override;

private:
    coal::Transform3s to_coal_transform(const gp_Trsf& trsf) const;

    std::shared_ptr<coal::CollisionGeometry> build_bvh(
        const std::vector<std::array<double, 3>>& vertices,
        const std::vector<std::array<int, 3>>&    triangles) const;

    // Build BVH from an already-tessellated MeshAsset; cached in mesh_geom_cache_.
    std::shared_ptr<coal::CollisionGeometry>
    get_or_build_geometry(const MeshAsset& mesh);

    // Tessellate `shape` on first use and build its BVH; cached in shape_geom_cache_.
    std::shared_ptr<coal::CollisionGeometry>
    get_or_build_geometry(const TopoDS_Shape& shape);

    double safety_margin_m_;

    // id → coal collision object (present objects only, rebuilt on sync)
    std::map<std::string, std::shared_ptr<coal::CollisionObject>> objects_;

    // MeshAsset raw pointer → coal geometry (avoids rebuilding BVH for unchanged meshes)
    std::map<const MeshAsset*, std::shared_ptr<coal::CollisionGeometry>> mesh_geom_cache_;

    // TopoDS_Shape raw pointer → coal geometry (avoids re-tessellating shapes
    // reused across many add_or_update calls, e.g. the nozzle)
    std::map<const TopoDS_Shape*, std::shared_ptr<coal::CollisionGeometry>> shape_geom_cache_;
};
