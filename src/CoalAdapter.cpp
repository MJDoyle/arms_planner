#include "assembler/CoalAdapter.hpp"
#include "assembler/MeshAsset.hpp"
#include "assembler/SceneModel.hpp"

#include <coal/collision.h>
#include <coal/distance.h>

#include <gp_Mat.hxx>
#include <gp_Trsf.hxx>
#include <gp_XYZ.hxx>

#include <limits>
#include <stdexcept>

CoalAdapter::CoalAdapter(double safety_margin_m)
    : safety_margin_m_(safety_margin_m)
{}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

coal::Transform3s CoalAdapter::to_coal_transform(const gp_Trsf& trsf) const
{
    const gp_Mat& rot = trsf.VectorialPart();
    coal::Matrix3s R;
    for (int i = 1; i <= 3; ++i)
        for (int j = 1; j <= 3; ++j)
            R(i - 1, j - 1) = rot.Value(i, j);

    const gp_XYZ& t = trsf.TranslationPart();
    coal::Vec3s T(t.X(), t.Y(), t.Z());

    return coal::Transform3s(R, T);
}

std::shared_ptr<coal::CollisionGeometry>
CoalAdapter::get_or_build_geometry(const MeshAsset& mesh)
{
    // Cache lookup by pointer identity — mesh assets are never mutated.
    auto it = geom_cache_.find(&mesh);
    if (it != geom_cache_.end())
        return it->second;

    // Build BVH from the shared mesh asset.
    std::vector<coal::Vec3s> pts;
    pts.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices)
        pts.emplace_back(v[0], v[1], v[2]);

    std::vector<coal::Triangle> tris;
    tris.reserve(mesh.triangles.size());
    for (const auto& t : mesh.triangles)
        tris.emplace_back(
            static_cast<size_t>(t[0]),
            static_cast<size_t>(t[1]),
            static_cast<size_t>(t[2]));

    auto model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
    model->beginModel(static_cast<int>(tris.size()),
                      static_cast<int>(pts.size()));
    model->addSubModel(pts, tris);
    model->endModel();

    geom_cache_[&mesh] = model;
    return model;
}

// ---------------------------------------------------------------------------
// CollisionAdapter interface
// ---------------------------------------------------------------------------

void CoalAdapter::add_or_update(const std::string& id,
                                 std::shared_ptr<MeshAsset> mesh,
                                 const gp_Trsf& pose)
{
    auto geom = get_or_build_geometry(*mesh);
    objects_[id] = std::make_shared<coal::CollisionObject>(geom, to_coal_transform(pose));
}

void CoalAdapter::remove(const std::string& id)
{
    objects_.erase(id);
}

void CoalAdapter::sync(const SceneModel& scene)
{
    objects_.clear();

    for (const auto& [id, obj] : scene.objects()) {
        if (!obj.present || !obj.mesh) continue;

        auto geom      = get_or_build_geometry(*obj.mesh);
        auto transform = to_coal_transform(obj.pose);
        objects_[id]   = std::make_shared<coal::CollisionObject>(geom, transform);
    }
}

bool CoalAdapter::collision_free(const std::string& id_a,
                                 const std::string& id_b) const
{
    auto it_a = objects_.find(id_a);
    auto it_b = objects_.find(id_b);
    if (it_a == objects_.end() || it_b == objects_.end())
        return true;  // absent object → no collision

    coal::CollisionRequest req;
    // Positive security_margin: report collision if shapes are within this
    // distance — biases errors toward false-positive (conservative, invariant 5).
    req.security_margin = safety_margin_m_;

    coal::CollisionResult res;
    coal::collide(it_a->second.get(), it_b->second.get(), req, res);
    return !res.isCollision();
}

double CoalAdapter::min_distance(const std::string& id_a,
                                 const std::string& id_b) const
{
    auto it_a = objects_.find(id_a);
    auto it_b = objects_.find(id_b);
    if (it_a == objects_.end() || it_b == objects_.end())
        return std::numeric_limits<double>::infinity();

    coal::DistanceRequest req;
    coal::DistanceResult res;
    coal::distance(it_a->second.get(), it_b->second.get(), req, res);
    return res.min_distance;
}
