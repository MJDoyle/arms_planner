#include "assembler/Tessellator.hpp"
#include "assembler/MeshFunctions.hpp"  // ShapeCentroid, BRepMesh_IncrementalMesh, etc.

#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>

namespace Tessellator {

std::shared_ptr<MeshAsset> tessellate(const TopoDS_Shape& shape, double deflection_mm)
{
    auto asset = std::make_shared<MeshAsset>();
    asset->linear_deflection_mm = deflection_mm;

    // Compute bbox centroid (same convention as ShapeSetCentroid / assembler_node cache).
    const gp_Pnt centroid = ShapeCentroid(shape);

    // Tessellate in-place on a copy so we don't perturb any existing mesh stored
    // on the caller's shape (e.g. if a finer mesh was applied by JigGenerator later).
    BRepMesh_IncrementalMesh mesher(shape, deflection_mm);
    mesher.Perform();

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        const int base = static_cast<int>(asset->vertices.size());
        const gp_Trsf trsf = loc.IsIdentity() ? gp_Trsf() : loc.Transformation();

        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i).Transformed(trsf);
            // Subtract bbox centroid → local frame; convert mm → m.
            asset->vertices.push_back({
                (p.X() - centroid.X()) * 0.001,
                (p.Y() - centroid.Y()) * 0.001,
                (p.Z() - centroid.Z()) * 0.001
            });
        }

        const bool reversed = (face.Orientation() == TopAbs_REVERSED);
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);
            asset->triangles.push_back({base + n1 - 1, base + n2 - 1, base + n3 - 1});
        }
    }

    return asset;
}

}  // namespace Tessellator
