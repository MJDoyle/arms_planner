/**
 * TestCradleGenerator
 *
 * Jig geometry:
 *   - Square base: bay_size × bay_size × 3 mm, 5 mm rounded corners.
 *   - 4 pegs (5 mm diameter) at the part's extreme edges in +X, -X, +Y, -Y.
 *     Each edge's most-lateral adjacent face drives the peg-top tilt angle (θ/2),
 *     where θ is that face's angle from Z.  The tilt axis is the edge tangent.
 *   - Part sits GAP mm above the base top; peg heights are set accordingly.
 */

#include "assembler/TestCradleGenerator.hpp"
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"
#include "assembler/ARMSConfig.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Geom_Plane.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <StlAPI_Writer.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

#include <algorithm>
#include <limits>
#include <sstream>
#include <iostream>
#include <vector>

// ---------------------------------------------------------------------------
// Rounded-rectangle solid (side × side × height, corner radius r).
// ---------------------------------------------------------------------------
static TopoDS_Shape makeRoundedRectSolid(double cx, double cy, double z_bot,
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
        return BRepPrimAPI_MakeBox(gp_Pnt(cx-h,cy-h,z_bot), side, side, height).Shape();
    };
    if (!mkW.IsDone()) return fallback();

    Handle(Geom_Plane) hPlane = new Geom_Plane(gp_Pnt(cx,cy,z_bot), gp_Dir(0,0,1));
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

    BRepPrimAPI_MakePrism prism(rf, gp_Vec(0,0,height));
    if (!prism.IsDone()) return fallback();
    return prism.Shape();
}

// ---------------------------------------------------------------------------
// Safe boolean helpers (static — avoids link conflict)
// ---------------------------------------------------------------------------
static TopoDS_Shape tFuse(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy=1e-4)
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

static TopoDS_Shape tCut(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy=1e-4)
{
    if (A.IsNull() || B.IsNull()) return A;
    try {
        TopTools_ListOfShape args, tools;
        args.Append(A); tools.Append(B);
        BRepAlgoAPI_Cut op;
        op.SetArguments(args); op.SetTools(tools);
        op.SetFuzzyValue(fuzzy); op.Build();
        if (!op.IsDone()) return A;
        const TopoDS_Shape r = op.Shape();
        return r.IsNull() ? A : r;
    } catch (...) { return A; }
}

// ===========================================================================
// TestCradleGenerator::createJig
// ===========================================================================
float TestCradleGenerator::createJig(float bay_size, int bay_index)
{
    RCLCPP_INFO(logger(), "TestCradleGenerator: %s  bay=%g #%d",
                name_.c_str(), (double)bay_size, bay_index);

    const double z_min = ShapeLowestPoint(shape_);
    const gp_Pnt part_cen = ShapeCentroid(shape_);
    const double cx = part_cen.X(), cy = part_cen.Y();
    const double bs = (double)bay_size;

    constexpr double FLOOR_H  = 3.0;
    constexpr double PEG_R    = 2.5;   // 5 mm diameter
    constexpr double CORNER_R = 5.0;
    constexpr double GAP      = 5.0;   // part bottom is GAP mm above base top
    constexpr double MIN_TILT = 30.0;  // minimum face angle from Z to apply tilt (°)

    // ---- Build edge-to-face map -----------------------------------------------
    TopTools_IndexedDataMapOfShapeListOfShape e2f;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, e2f);

    // ---- Collect every edge midpoint with its most-lateral face info ----------
    struct EdgeInfo {
        double x = 0, y = 0, z = 0;
        double face_angle_deg = 0;  // angle of most-lateral face from Z axis
        gp_Vec face_normal;         // outward normal of that face
        gp_Vec tangent;             // edge tangent at midpoint (unit)
    };
    std::vector<EdgeInfo> all_edges;
    all_edges.reserve(64);

    // Laterality = sin(angle from Z) = horizontal magnitude of outward normal
    auto laterality = [&](const TopoDS_Face& f) -> double {
        try {
            gp_Dir n = outwardFaceNormal(f);
            return std::sqrt(n.X()*n.X() + n.Y()*n.Y());
        } catch (...) { return 0.0; }
    };

    for (TopExp_Explorer ex(shape_, TopAbs_EDGE); ex.More(); ex.Next()) {
        try {
            const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());

            BRepAdaptor_Curve adaptor(edge);
            const double mid = (adaptor.FirstParameter() + adaptor.LastParameter()) * 0.5;
            gp_Pnt mp; gp_Vec tang;
            adaptor.D1(mid, mp, tang);
            if (tang.Magnitude() < 1e-10) continue;
            tang.Normalize();

            const int idx = e2f.FindIndex(edge);
            if (idx < 1) continue;
            const TopTools_ListOfShape& fl = e2f(idx);
            if (fl.Extent() < 2) continue;

            TopTools_ListIteratorOfListOfShape it(fl);
            const TopoDS_Face& fa = TopoDS::Face(it.Value()); it.Next();
            const TopoDS_Face& fb = TopoDS::Face(it.Value());

            // Use the more lateral (more vertical) of the two adjacent faces
            const TopoDS_Face& primary = (laterality(fa) >= laterality(fb)) ? fa : fb;

            double angle_deg = 0;
            gp_Vec fn_vec;
            try {
                gp_Dir n = outwardFaceNormal(primary);
                fn_vec = gp_Vec(n);
                angle_deg = std::acos(std::min(1.0, std::abs(n.Z()))) * 180.0 / M_PI;
            } catch (...) {}

            all_edges.push_back({mp.X(), mp.Y(), mp.Z(), angle_deg, fn_vec, tang});
        } catch (...) {}
    }

    if (all_edges.empty()) {
        RCLCPP_ERROR(logger(), "TestCradleGenerator: no edges found");
        return 0.0f;
    }

    // ---- Find most-extreme edge in a direction (lowest-Z tiebreaker) ----------
    // No face filtering — always return the most extreme position so maxX ≠ minX.
    auto bestEdge = [&](int dim, bool want_max) -> const EdgeInfo* {
        const EdgeInfo* best = nullptr;
        for (const auto& e : all_edges) {
            if (!best) { best = &e; continue; }
            double ve = (dim == 0) ? e.x   : e.y;
            double vb = (dim == 0) ? best->x : best->y;
            if (want_max ? (ve > vb + 1e-6) : (ve < vb - 1e-6)) {
                best = &e;
            } else if (std::abs(ve - vb) < 1e-3 && e.z < best->z) {
                best = &e; // prefer lower Z when tied in primary direction
            }
        }
        return best;
    };

    struct Peg {
        double x, y, z, face_angle_deg;
        gp_Vec face_normal, tangent;
        bool valid;
    };
    auto makePeg = [&](int dim, bool want_max) -> Peg {
        const EdgeInfo* e = bestEdge(dim, want_max);
        if (!e) return {0,0,0,0,{},{},false};
        return {e->x, e->y, e->z, e->face_angle_deg, e->face_normal, e->tangent, true};
    };

    const Peg pegs[4] = {
        makePeg(0, true),   // max X
        makePeg(0, false),  // min X
        makePeg(1, true),   // max Y
        makePeg(1, false),  // min Y
    };
    const char* labels[4] = {"maxX","minX","maxY","minY"};

    // ---- Square base ---------------------------------------------------------
    TopoDS_Shape jig = makeRoundedRectSolid(cx, cy, z_min - FLOOR_H, bs, CORNER_R, FLOOR_H);
    if (jig.IsNull()) { RCLCPP_ERROR(logger(), "Base failed"); return 0.0f; }

    // ---- Cylinders with optionally tilted tops --------------------------------
    for (int i = 0; i < 4; ++i) {
        try {
            const Peg& p = pegs[i];
            if (!p.valid) { RCLCPP_WARN(logger(), "  peg %s: no edge", labels[i]); continue; }

            const double cyl_h     = (p.z - z_min) + GAP;
            const double cyl_h_raw = cyl_h + 30.0; // overshoot; tilt cut trims to cyl_h

            const gp_Ax2 axis(gp_Pnt(p.x, p.y, z_min), gp_Dir(0,0,1));
            TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(axis, PEG_R, cyl_h_raw).Shape();

            // --- Tilt the top face using a rotated box cut ---------------------
            const double tilt_deg = p.face_angle_deg / 2.0;
            const double tilt_rad = tilt_deg * M_PI / 180.0;

            if (tilt_rad > 1e-6 && p.face_normal.Magnitude() > 1e-10
                && p.face_angle_deg >= MIN_TILT)
            {
                try {
                    gp_Dir tilt_axis(p.tangent);

                    // Rotation sign: rotate Z toward face_normal around tilt_axis.
                    // cross(Z, face_normal) · tilt_axis > 0 → positive rotation.
                    gp_Vec cross_Z_n = gp_Vec(0,0,1).Crossed(p.face_normal);
                    const double align = (cross_Z_n.Magnitude() > 1e-10)
                                         ? cross_Z_n.Dot(gp_Vec(tilt_axis)) : 0.0;
                    const double rot_sign = (align >= 0) ? -1.0 : 1.0;

                    // Build cutting box: large slab, bottom face at Z=0 in local coords.
                    // Rotating by tilt_rad around tilt_axis through origin makes the
                    // bottom face become the tilted cutting plane.
                    // Translating by top_cen places the plane at the desired height.
                    constexpr double SLAB = 60.0;
                    TopoDS_Shape cutter = BRepPrimAPI_MakeBox(
                        gp_Pnt(-SLAB, -SLAB, 0.0), 2*SLAB, 2*SLAB, SLAB).Shape();

                    gp_Trsf rot;
                    rot.SetRotation(gp_Ax1(gp_Pnt(0,0,0), tilt_axis), rot_sign * tilt_rad);
                    gp_Trsf trans;
                    trans.SetTranslation(gp_Vec(p.x, p.y, z_min + cyl_h));
                    const gp_Trsf combined = trans * rot;

                    BRepBuilderAPI_Transform mkT(cutter, combined, Standard_True);
                    if (mkT.IsDone()) {
                        TopoDS_Shape cut = tCut(cyl, mkT.Shape());
                        if (!cut.IsNull()) cyl = cut;
                    }
                } catch (...) {
                    RCLCPP_WARN(logger(), "  peg %s: tilt cut failed, flat top", labels[i]);
                }
            }

            jig = tFuse(jig, cyl);
            RCLCPP_INFO(logger(), "  peg %s at (%.2f,%.2f) h=%.2f tilt=%.1f°",
                        labels[i], p.x, p.y, cyl_h, tilt_deg);
        } catch (...) {
            RCLCPP_WARN(logger(), "  peg %s: construction failed, skipping", labels[i]);
        }
    }

    // ---- z-offset + translate + export ----------------------------------------
    const float jig_part_z_offset =
        static_cast<float>(ShapeCentroid(shape_).Z() - ShapeCentroid(jig).Z())
        + static_cast<float>(GAP);

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
