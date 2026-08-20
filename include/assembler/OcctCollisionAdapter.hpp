#pragma once

#include "assembler/CollisionAdapter.hpp"

#include <Bnd_Box.hxx>
#include <TopoDS_Shape.hxx>

#include <map>
#include <memory>
#include <string>

class OcctCollisionAdapter : public CollisionAdapter {
public:
    explicit OcctCollisionAdapter(double safety_margin_m);

    void sync(const SceneModel& scene) override;

    void add_or_update(const std::string& id,
                       std::shared_ptr<TopoDS_Shape> shape,
                       const gp_Trsf& pose) override;

    void remove(const std::string& id) override;

    bool collision_free(const std::string& id_a,
                        const std::string& id_b) const override;

    double min_distance(const std::string& id_a,
                        const std::string& id_b) const override;

private:
    double safety_margin_m_;

    // id → world-space shape (local-frame shape with pose already applied)
    std::map<std::string, TopoDS_Shape> objects_;
    // id → AABB for quick rejection before exact distance query
    std::map<std::string, Bnd_Box>      bboxes_;
};
