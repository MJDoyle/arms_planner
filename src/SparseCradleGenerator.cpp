/**
 * SparseCradleGenerator
 *
 * Jig geometry:
 *   - Square base: bay_size × bay_size × 5 mm, 5 mm rounded corners.
 *   - Part sits 5 mm above the base top.
 *   - 4 straight walls (5 mm thick, bay_size long, part_height + 5 mm tall)
 *     flush with the outer faces of the base plate, rising from the base top.
 */

#include "assembler/SparseCradleGenerator.hpp"
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"
#include "assembler/ARMSConfig.hpp"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Geom_Plane.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <StlAPI_Writer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

#include <algorithm>
#include <sstream>

// ---------------------------------------------------------------------------
// Rounded-rectangle solid (side × side × height, corner radius r).
// ---------------------------------------------------------------------------
static TopoDS_Shape scgMakeRoundedRectSolid(double cx, double cy, double z_bot,
                                             double side, double corner_r, double height)
{
    const double h = side / 2.0;
    BRepBuilderAPI_MakePolygon mkW;
    mkW.Add(gp_Pnt(cx - h, cy - h, z_bot));
    mkW.Add(gp_Pnt(cx + h, cy - h, z_bot));
    mkW.Add(gp_Pnt(cx + h, cy + h, z_bot));
    mkW.Add(gp_Pnt(cx - h, cy + h, z_bot));
    mkW.Close();

    auto fallback = [&]() {
        return BRepPrimAPI_MakeBox(gp_Pnt(cx-h, cy-h, z_bot), side, side, height).Shape();
    };
    if (!mkW.IsDone()) return fallback();

    Handle(Geom_Plane) hPlane = new Geom_Plane(gp_Pnt(cx, cy, z_bot), gp_Dir(0, 0, 1));
    BRepBuilderAPI_MakeFace mkF(hPlane, mkW.Wire(), Standard_True);
    if (!mkF.IsDone()) return fallback();
    TopoDS_Face face = mkF.Face();

    BRepFilletAPI_MakeFillet2d mk2d(face);
    {
        TopTools_IndexedMapOfShape vmap;
        TopExp::MapShapes(face, TopAbs_VERTEX, vmap);
        const double r = std::min(corner_r, h - 0.5);
        for (int i = 1; i <= vmap.Extent(); ++i)
            mk2d.AddFillet(TopoDS::Vertex(vmap(i)), r);
    }
    mk2d.Build();
    TopoDS_Shape rf = mk2d.IsDone() ? mk2d.Shape() : face;

    BRepPrimAPI_MakePrism prism(rf, gp_Vec(0, 0, height));
    if (!prism.IsDone()) return fallback();
    return prism.Shape();
}

// ---------------------------------------------------------------------------
static TopoDS_Shape scgFuse(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy = 1e-4)
{
    if (A.IsNull()) return B;
    if (B.IsNull()) return A;
    try {
        TopTools_ListOfShape args, tools;
        args.Append(A); tools.Append(B);
        BRepAlgoAPI_Fuse op;
        op.SetArguments(args); op.SetTools(tools);
        op.SetFuzzyValue(fuzzy); op.Build();
        if (!op.IsDone()) return A;
        const TopoDS_Shape r = op.Shape();
        return r.IsNull() ? A : r;
    } catch (...) { return A; }
}

// ===========================================================================
// SparseCradleGenerator::createJig
// ===========================================================================
float SparseCradleGenerator::createJig(float bay_size, int bay_index)
{
    RCLCPP_INFO(logger(), "SparseCradleGenerator: %s  bay=%g #%d",
                name_.c_str(), (double)bay_size, bay_index);

    const double z_min = ShapeLowestPoint(shape_);
    const double z_max = ShapeHighestPoint(shape_);
    const gp_Pnt part_cen = ShapeCentroid(shape_);
    const double cx = part_cen.X(), cy = part_cen.Y();
    const double bs = (double)bay_size;

    constexpr double FLOOR_H  = 5.0;
    constexpr double GAP      = 5.0;
    constexpr double WALL_T   = 5.0;
    constexpr double CORNER_R = 5.0;

    const double base_bot = z_min - GAP - FLOOR_H;
    const double base_top = z_min - GAP;
    const double part_h   = z_max - z_min;
    const double wall_h   = part_h + GAP;

    // ---- Base plate ---------------------------------------------------------
    TopoDS_Shape jig = scgMakeRoundedRectSolid(cx, cy, base_bot, bs, CORNER_R, FLOOR_H);
    if (jig.IsNull()) { RCLCPP_ERROR(logger(), "SparseCradleGenerator: base failed"); return 0.0f; }

    // ---- Find contact positions via part bounding box -----------------------
    // For a flat wall moving in a cardinal direction, the first contact with the
    // part occurs at the part's extreme extent in that direction.  The wall
    // midplane is placed at that extent so the contact point sits 2.5 mm into
    // the wall from each face.
    Bnd_Box bbox;
    BRepBndLib::AddOptimal(shape_, bbox, Standard_False, Standard_False);
    double bb_xmin, bb_ymin, bb_zmin_bb, bb_xmax, bb_ymax, bb_zmax_bb;
    bbox.Get(bb_xmin, bb_ymin, bb_zmin_bb, bb_xmax, bb_ymax, bb_zmax_bb);

    RCLCPP_INFO(logger(), "  part bbox  x[%.2f,%.2f]  y[%.2f,%.2f]",
                bb_xmin, bb_xmax, bb_ymin, bb_ymax);

    // ---- Find the 3D contact point for each wall ----------------------------
    // Place a query vertex very far away in the wall's outward direction so that
    // BRepExtrema is forced to return the extremal surface point on the part.
    // That point's z sets the wall's height (top of wall = contact z).
    const double FAR = 1.0e6;

    auto contactZ = [&](gp_Pnt far_pt) -> double {
        BRepBuilderAPI_MakeVertex vtx(far_pt);
        if (!vtx.IsDone()) return z_max;
        BRepExtrema_DistShapeShape dist(shape_, vtx.Shape());
        if (!dist.IsDone() || dist.NbSolution() == 0) return z_max;
        return dist.PointOnShape1(1).Z();
    };

    const double cz_px = contactZ(gp_Pnt(bb_xmax + FAR, cy,       z_min));
    const double cz_mx = contactZ(gp_Pnt(bb_xmin - FAR, cy,       z_min));
    const double cz_py = contactZ(gp_Pnt(cx,       bb_ymax + FAR, z_min));
    const double cz_my = contactZ(gp_Pnt(cx,       bb_ymin - FAR, z_min));

    // Wall height = from base_top up to the contact z.  Clamp to at least GAP.
    auto wallH = [&](double cz) { return std::max(GAP, cz - base_top); };

    // ---- Four walls ---------------------------------------------------------
    // Midplane in the normal direction = contact extent.
    // Height is per-wall so each wall's top centre matches the part surface there.

    // +X wall: contact at bb_xmax, midplane at bb_xmax
    TopoDS_Shape wall_px = BRepPrimAPI_MakeBox(
        gp_Pnt(bb_xmax - WALL_T/2.0, cy - bs/2.0, base_top),
        WALL_T, bs, wallH(cz_px)).Shape();

    // -X wall: contact at bb_xmin, midplane at bb_xmin
    TopoDS_Shape wall_mx = BRepPrimAPI_MakeBox(
        gp_Pnt(bb_xmin - WALL_T/2.0, cy - bs/2.0, base_top),
        WALL_T, bs, wallH(cz_mx)).Shape();

    // +Y wall: contact at bb_ymax, midplane at bb_ymax
    TopoDS_Shape wall_py = BRepPrimAPI_MakeBox(
        gp_Pnt(cx - bs/2.0, bb_ymax - WALL_T/2.0, base_top),
        bs, WALL_T, wallH(cz_py)).Shape();

    // -Y wall: contact at bb_ymin, midplane at bb_ymin
    TopoDS_Shape wall_my = BRepPrimAPI_MakeBox(
        gp_Pnt(cx - bs/2.0, bb_ymin - WALL_T/2.0, base_top),
        bs, WALL_T, wallH(cz_my)).Shape();

    jig = scgFuse(jig, wall_px);
    jig = scgFuse(jig, wall_mx);
    jig = scgFuse(jig, wall_py);
    jig = scgFuse(jig, wall_my);

    RCLCPP_INFO(logger(), "  base %.0fx%.0f  gap=%.0f  contact z: +x=%.2f -x=%.2f +y=%.2f -y=%.2f",
                bs, bs, GAP, cz_px, cz_mx, cz_py, cz_my);

    // ---- Translate to JIG_CENTER_Z and export --------------------------------
    const float jig_part_z_offset =
        static_cast<float>(ShapeCentroid(shape_).Z() - ShapeCentroid(jig).Z());

    {
        const gp_Pnt c = ShapeCentroid(jig);
        jig = ShapeSetCentroid(jig, gp_Pnt(c.X(), c.Y(), JIG_CENTER_Z));
    }

    BRepMesh_IncrementalMesh(jig, 0.01).Perform();

    std::stringstream ss;
    ss << OUTPUT_DIR << "jig_" << name_ << "_size_" << bay_size
       << "_index_" << bay_index << ".stl";
    StlAPI_Writer().Write(jig, ss.str().c_str());

    RCLCPP_INFO(logger(), "  exported %s  (z_offset=%.3f)", ss.str().c_str(), jig_part_z_offset);
    return jig_part_z_offset;
}
