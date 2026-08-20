#include "assembler/OcctCollisionAdapter.hpp"
#include "assembler/SceneModel.hpp"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>

#include <limits>

OcctCollisionAdapter::OcctCollisionAdapter(double safety_margin_m)
    : safety_margin_m_(safety_margin_m)
{}

void OcctCollisionAdapter::add_or_update(const std::string& id,
                                          std::shared_ptr<TopoDS_Shape> local_shape,
                                          const gp_Trsf& pose)
{
    BRepBuilderAPI_Transform xform(*local_shape, pose, /*copy=*/true);
    objects_[id] = xform.Shape();

    Bnd_Box bbox;
    BRepBndLib::Add(objects_[id], bbox);
    bboxes_[id] = bbox;
}

void OcctCollisionAdapter::remove(const std::string& id)
{
    objects_.erase(id);
    bboxes_.erase(id);
}

void OcctCollisionAdapter::sync(const SceneModel& scene)
{
    objects_.clear();
    bboxes_.clear();

    for (const auto& [id, obj] : scene.objects()) {
        if (!obj.present || !obj.shape) continue;
        add_or_update(id, obj.shape, obj.pose);
    }
}

bool OcctCollisionAdapter::collision_free(const std::string& id_a,
                                           const std::string& id_b) const
{
    auto it_a = objects_.find(id_a);
    auto it_b = objects_.find(id_b);
    if (it_a == objects_.end() || it_b == objects_.end())
        return true;  // absent object → no collision

    if (bboxes_.at(id_a).IsOut(bboxes_.at(id_b)))
        return true;  // AABB rejection

    BRepExtrema_DistShapeShape dist(it_a->second, it_b->second);
    if (!dist.IsDone())
        return false;  // conservative: assume collision

    return dist.Value() >= safety_margin_m_;
}

double OcctCollisionAdapter::min_distance(const std::string& id_a,
                                           const std::string& id_b) const
{
    auto it_a = objects_.find(id_a);
    auto it_b = objects_.find(id_b);
    if (it_a == objects_.end() || it_b == objects_.end())
        return std::numeric_limits<double>::infinity();

    BRepExtrema_DistShapeShape dist(it_a->second, it_b->second);
    if (!dist.IsDone())
        return 0.0;  // conservative

    return dist.Value();
}
