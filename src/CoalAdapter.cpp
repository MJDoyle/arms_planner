#include "assembler/CoalAdapter.hpp"
#include "assembler/ARMSConfig.hpp"
#include "assembler/MeshAsset.hpp"
#include "assembler/SceneModel.hpp"

#include <coal/collision.h>
#include <coal/distance.h>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Mat.hxx>
#include <gp_Trsf.hxx>
#include <gp_XYZ.hxx>

#include <limits>
#include <stdexcept>
#include <utility>

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
CoalAdapter::build_bvh(const std::vector<std::array<double, 3>>& vertices,
                        const std::vector<std::array<int, 3>>&    triangles) const
{
    std::vector<coal::Vec3s> pts;
    pts.reserve(vertices.size());
    for (const auto& v : vertices)
        pts.emplace_back(v[0], v[1], v[2]);

    std::vector<coal::Triangle> tris;
    tris.reserve(triangles.size());
    for (const auto& t : triangles)
        tris.emplace_back(
            static_cast<size_t>(t[0]),
            static_cast<size_t>(t[1]),
            static_cast<size_t>(t[2]));

    auto model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
    model->beginModel(static_cast<int>(tris.size()),
                      static_cast<int>(pts.size()));
    model->addSubModel(pts, tris);
    model->endModel();
    return model;
}

std::shared_ptr<coal::CollisionGeometry>
CoalAdapter::get_or_build_geometry(const MeshAsset& mesh)
{
    // Cache lookup by pointer identity — mesh assets are never mutated.
    auto it = mesh_geom_cache_.find(&mesh);
    if (it != mesh_geom_cache_.end())
        return it->second;

    auto model = build_bvh(mesh.vertices, mesh.triangles);
    mesh_geom_cache_[&mesh] = model;
    return model;
}

std::shared_ptr<coal::CollisionGeometry>
CoalAdapter::get_or_build_geometry(const TopoDS_Shape& shape)
{
    // Cache lookup by pointer identity — callers (VacuumGraspGenerator, the
    // z-lift check) reuse the same shared_ptr across many add_or_update
    // calls with only the pose changing, so this tessellates each transient
    // shape (nozzle, lifted-part copy) exactly once.
    auto it = shape_geom_cache_.find(&shape);
    if (it != shape_geom_cache_.end())
        return it->second;

    // NOTE: deliberately NOT Tessellator::tessellate() here — that function
    // expects a raw millimetre shape and itself converts mm -> m and
    // re-centres on the bbox centroid.  Every shape reaching add_or_update
    // (nozzle, lifted-part copies) is already LocalFrameShapeM()-converted,
    // i.e. already local-frame metres.  Running it through tessellate() a
    // second time re-divided coordinates by 1000 again, shrinking the
    // collision geometry to ~1/1000th its real size while leaving the pose
    // correct — every collision check then silently missed real overlaps.
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<int, 3>>    triangles;

    // Deflection must be in the same units as `shape` — already metres here,
    // unlike the mm-based MESH_DEFLECTION_MM constant used elsewhere.
    BRepMesh_IncrementalMesh mesher(shape, MESH_DEFLECTION_MM * 0.001);
    mesher.Perform();

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        const int base = static_cast<int>(vertices.size());
        const gp_Trsf trsf = loc.IsIdentity() ? gp_Trsf() : loc.Transformation();

        for (int i = 1; i <= tri->NbNodes(); ++i) {
            const gp_Pnt p = tri->Node(i).Transformed(trsf);
            vertices.push_back({p.X(), p.Y(), p.Z()});
        }

        const bool reversed = (face.Orientation() == TopAbs_REVERSED);
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);
            triangles.push_back({base + n1 - 1, base + n2 - 1, base + n3 - 1});
        }
    }

    auto model = build_bvh(vertices, triangles);
    shape_geom_cache_[&shape] = model;
    return model;
}

// ---------------------------------------------------------------------------
// CollisionAdapter interface
// ---------------------------------------------------------------------------

void CoalAdapter::add_or_update(const std::string& id,
                                 std::shared_ptr<TopoDS_Shape> shape,
                                 const gp_Trsf& pose)
{
    auto geom = get_or_build_geometry(*shape);
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
