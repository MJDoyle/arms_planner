#include "assembler/SceneModel.hpp"

#include <stdexcept>

void SceneModel::add_object(const std::string& id,
                            std::shared_ptr<MeshAsset> mesh,
                            SceneRole role,
                            const gp_Trsf& pose,
                            bool present)
{
    objects_[id] = SceneObject{mesh, role, pose, present};
}

void SceneModel::remove_object(const std::string& id)
{
    objects_.erase(id);
}

void SceneModel::set_present(const std::string& id, bool present)
{
    auto it = objects_.find(id);
    if (it != objects_.end())
        it->second.present = present;
}

void SceneModel::set_pose(const std::string& id, const gp_Trsf& pose)
{
    auto it = objects_.find(id);
    if (it != objects_.end())
        it->second.pose = pose;
}

bool SceneModel::has_object(const std::string& id) const
{
    return objects_.count(id) > 0;
}

const SceneObject& SceneModel::get_object(const std::string& id) const
{
    auto it = objects_.find(id);
    if (it == objects_.end())
        throw std::out_of_range("SceneModel: unknown object id: " + id);
    return it->second;
}

std::vector<std::string> SceneModel::present_object_ids() const
{
    std::vector<std::string> ids;
    for (const auto& [id, obj] : objects_)
        if (obj.present)
            ids.push_back(id);
    return ids;
}
