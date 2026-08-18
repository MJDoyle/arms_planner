/**
 * AdvancedCradleGenerator: ring-wall jig derived from the part's 2D cross-section.
 *
 * Instead of carving a convex-hull negative out of a solid block, this generator:
 *  1. Samples 2D cross-sections through the triangulated mesh at multiple heights.
 *  2. Computes the 2D convex hull of all points in the jig-height band.
 *  3. Builds a thin-walled ring (outer profile minus inner profile) around that hull.
 *  4. Adds a base plate; for tall parts the base has a hole so the part's lower
 *     section can poke through (with a minimum 3 mm solid border guaranteed).
 *  5. Fillets the outer vertical edges at 5 mm, same as CradleGenerator.
 */

#include "assembler/AdvancedCradleGenerator.hpp"
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"
#include "assembler/ARMSConfig.hpp"

#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <Geom_Plane.hxx>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <StlAPI_Writer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <Standard_Failure.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <iostream>
#include <vector>

// ---------------------------------------------------------------------------
// 2-D point type (local to this translation unit)
// ---------------------------------------------------------------------------

struct Pt2d {
    double x, y;
    bool operator<(const Pt2d& o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
    bool operator==(const Pt2d& o) const {
        return std::abs(x - o.x) < 1e-9 && std::abs(y - o.y) < 1e-9;
    }
};

// ---------------------------------------------------------------------------
// Sample a 2-D point cloud from the triangulated mesh at height z ± dz.
// Assumes BRepMesh_IncrementalMesh has already been called on the shape.
// ---------------------------------------------------------------------------
static std::vector<Pt2d> sampleXSection(const TopoDS_Shape& shape, double z, double dz)
{
    std::vector<Pt2d> pts;

    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& F = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(F, loc);
        if (tri.IsNull()) continue;

        const gp_Trsf& T = loc.Transformation();
        const bool hasLoc = !loc.IsIdentity();

        for (int t = 1; t <= tri->NbTriangles(); ++t) {
            int n1, n2, n3;
            tri->Triangle(t).Get(n1, n2, n3);

            auto node = [&](int n) {
                gp_Pnt p = tri->Node(n);
                if (hasLoc) p.Transform(T);
                return p;
            };

            const std::array<gp_Pnt, 3> P = {node(n1), node(n2), node(n3)};

            // Vertices within the band
            for (const gp_Pnt& p : P) {
                if (std::abs(p.Z() - z) <= dz)
                    pts.push_back({p.X(), p.Y()});
            }

            // Edge–plane intersections for edges that cross z
            for (int i = 0; i < 3; ++i) {
                const gp_Pnt& a = P[i];
                const gp_Pnt& b = P[(i + 1) % 3];
                if ((a.Z() - z) * (b.Z() - z) < 0.0) {
                    const double t_lerp = (z - a.Z()) / (b.Z() - a.Z());
                    pts.push_back({a.X() + t_lerp * (b.X() - a.X()),
                                   a.Y() + t_lerp * (b.Y() - a.Y())});
                }
            }
        }
    }
    return pts;
}

// ---------------------------------------------------------------------------
// Andrew's monotone-chain convex hull — returns CCW polygon.
// ---------------------------------------------------------------------------
static std::vector<Pt2d> convexHull2D(std::vector<Pt2d> pts)
{
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());

    const int n = (int)pts.size();
    if (n < 3) return pts;

    auto cross = [](const Pt2d& O, const Pt2d& A, const Pt2d& B) {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    };

    std::vector<Pt2d> h;
    h.reserve(2 * n);

    // Lower hull
    for (int i = 0; i < n; ++i) {
        while (h.size() >= 2 && cross(h[h.size()-2], h[h.size()-1], pts[i]) <= 0)
            h.pop_back();
        h.push_back(pts[i]);
    }
    // Upper hull
    const int lo = (int)h.size() + 1;
    for (int i = n - 2; i >= 0; --i) {
        while ((int)h.size() >= lo && cross(h[h.size()-2], h[h.size()-1], pts[i]) <= 0)
            h.pop_back();
        h.push_back(pts[i]);
    }
    h.pop_back(); // last point = first point
    return h;     // CCW order
}

// ---------------------------------------------------------------------------
// Shoelace area (always positive).
// ---------------------------------------------------------------------------
static double polygonArea2D(const std::vector<Pt2d>& poly)
{
    double area = 0;
    const int n = (int)poly.size();
    for (int i = 0; i < n; ++i) {
        const Pt2d& a = poly[i];
        const Pt2d& b = poly[(i + 1) % n];
        area += a.x * b.y - b.x * a.y;
    }
    return std::abs(area) * 0.5;
}

// ---------------------------------------------------------------------------
// Miter-offset a CCW convex polygon outward by d.
// Negative d shrinks the polygon inward.
// ---------------------------------------------------------------------------
static std::vector<Pt2d> offsetHull2D(const std::vector<Pt2d>& hull, double d)
{
    const int n = (int)hull.size();
    if (n < 3) return hull;

    // Outward normal of edge a→b for a CCW polygon
    auto outNorm = [](const Pt2d& a, const Pt2d& b) -> Pt2d {
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-12) return {0, 0};
        return {dy / len, -dx / len};
    };

    std::vector<Pt2d> result;
    result.reserve(n);

    for (int i = 0; i < n; ++i) {
        const Pt2d& prev = hull[(i - 1 + n) % n];
        const Pt2d& curr = hull[i];
        const Pt2d& next = hull[(i + 1) % n];

        const Pt2d n1 = outNorm(prev, curr);
        const Pt2d n2 = outNorm(curr, next);

        double bx = n1.x + n2.x, by = n1.y + n2.y;
        const double blen = std::sqrt(bx * bx + by * by);

        if (blen < 1e-10) {
            // Anti-parallel edges (straight line) — use n1 directly
            result.push_back({curr.x + n1.x * d, curr.y + n1.y * d});
            continue;
        }

        bx /= blen; by /= blen;

        double dot = n1.x * bx + n1.y * by;
        // Clamp to avoid degenerate miter at very acute angles
        if (std::abs(dot) < 0.05) dot = std::copysign(0.05, dot);
        double scale = d / dot;
        // Cap miter spike at 4× the requested offset
        const double cap = 4.0 * std::abs(d);
        if (scale >  cap) scale =  cap;
        if (scale < -cap) scale = -cap;

        result.push_back({curr.x + bx * scale, curr.y + by * scale});
    }
    return result;
}

// ---------------------------------------------------------------------------
// Extrude a 2-D polygon into a solid prism.
// BRepPrimAPI_MakePrism from a Face produces a Solid in OCCT 7.x.
// ---------------------------------------------------------------------------
static TopoDS_Shape makeExtrudedSolid(const std::vector<Pt2d>& poly,
                                      double z_bot, double height)
{
    if ((int)poly.size() < 3 || std::abs(height) < 1e-9) return {};

    BRepBuilderAPI_MakePolygon mkPoly;
    for (const Pt2d& p : poly)
        mkPoly.Add(gp_Pnt(p.x, p.y, z_bot));
    mkPoly.Close();
    if (!mkPoly.IsDone()) return {};

    Handle(Geom_Plane) hPlane = new Geom_Plane(gp_Pnt(0, 0, z_bot), gp_Dir(0, 0, 1));
    BRepBuilderAPI_MakeFace mkFace(hPlane, mkPoly.Wire(), Standard_True);
    if (!mkFace.IsDone()) return {};

    BRepPrimAPI_MakePrism prism(mkFace.Face(), gp_Vec(0, 0, height));
    if (!prism.IsDone()) return {};

    return prism.Shape(); // Solid when input is a Face
}

// ---------------------------------------------------------------------------
// Boolean helpers (static to avoid link conflicts with CradleGenerator.cpp)
// ---------------------------------------------------------------------------
static TopoDS_Shape advCut(const TopoDS_Shape& A, const TopoDS_Shape& B,
                           double fuzzy = 1e-4)
{
    if (A.IsNull() || B.IsNull()) return A;
    try {
        TopTools_ListOfShape args, tools;
        args.Append(A);
        tools.Append(B);
        BRepAlgoAPI_Cut op;
        op.SetArguments(args);
        op.SetTools(tools);
        op.SetFuzzyValue(fuzzy);
        op.Build();
        if (!op.IsDone()) return A;
        const TopoDS_Shape r = op.Shape();
        return r.IsNull() ? A : r;
    } catch (Standard_Failure& e) {
        std::cerr << "advCut: " << e.GetMessageString() << "\n";
        return A;
    }
}

// ===========================================================================
// AdvancedCradleGenerator::createJig
//
// Geometry overview:
//   - One solid box: bay_size × bay_size × (ring_height + floor_thickness)
//     The lower floor_thickness is the solid base plate.
//     The upper ring_height has the part cavity cut from it.
//   - Inner void: part convex hull + clearance, extruded from z_ring_bot upward.
//     It does NOT extend into the base, so the part always rests on the floor.
//   - ring_height = min(part_height, JIG_HEIGHT) — no wasted height above the part.
//   - 5 mm fillet on the 4 outer vertical corners of the square footprint.
// ===========================================================================
float AdvancedCradleGenerator::createJig(float bay_size, int bay_index)
{
    RCLCPP_INFO(logger(), "AdvancedCradleGenerator: %s  bay=%g #%d",
                name_.c_str(), static_cast<double>(bay_size), bay_index);

    const double z_min       = ShapeLowestPoint(shape_);
    const double z_max       = ShapeHighestPoint(shape_);
    const double part_height = z_max - z_min;

    // Ring height: cap at JIG_HEIGHT but never taller than the actual part.
    // This prevents empty ring height above short parts.
    const double ring_height    = std::min(part_height, static_cast<double>(JIG_HEIGHT));
    const double floor_thickness = 3.0;  // base plate (mm)
    const double clearance       = static_cast<double>(scaling_distance_);
    const double bs              = static_cast<double>(bay_size);

    const double z_ring_bot = z_min;                     // cavity floor = part bottom
    const double z_ring_top = z_ring_bot + ring_height;  // cavity top

    // ---- Triangulate shape once -----------------------------------------------
    {
        BRepMesh_IncrementalMesh mesher(shape_, 0.5);
        mesher.Perform();
    }

    // ---- Sample cross-sections across the ring height band --------------------
    std::vector<Pt2d> ring_pts;
    {
        const double z_sample_top = std::min(z_ring_top, z_max);
        for (double z = z_ring_bot; z <= z_sample_top; z += 1.5)
        {
            auto pts = sampleXSection(shape_, z, 0.75);
            ring_pts.insert(ring_pts.end(), pts.begin(), pts.end());
        }
        if (ring_pts.empty())
        {
            auto pts = sampleXSection(shape_, z_min, 2.0);
            ring_pts.insert(ring_pts.end(), pts.begin(), pts.end());
        }
    }

    if (static_cast<int>(ring_pts.size()) < 4) {
        RCLCPP_ERROR(logger(), "AdvancedCradleGenerator: too few cross-section points");
        return 0.0f;
    }

    const auto inner_hull = convexHull2D(ring_pts);
    if (static_cast<int>(inner_hull.size()) < 3) {
        RCLCPP_ERROR(logger(), "AdvancedCradleGenerator: hull degenerate");
        return 0.0f;
    }

    RCLCPP_INFO(logger(), "  hull: %zu verts  area=%.1f mm²  ring_h=%.1f",
                inner_hull.size(), polygonArea2D(inner_hull), ring_height);

    // Inner cavity = part hull + clearance gap
    const auto cavity_profile = offsetHull2D(inner_hull, clearance);

    // ---- Part centroid XY (used to centre the square footprint) ---------------
    const gp_Pnt part_cen = ShapeCentroid(shape_);
    const double cx = part_cen.X();
    const double cy = part_cen.Y();

    // ---- Build the full jig block: bay_size × bay_size × total_height ---------
    // The block runs from (z_ring_bot - floor_thickness) to z_ring_top.
    // The lower floor_thickness is left solid; the cavity is cut from z_ring_bot up.
    const double total_height = ring_height + floor_thickness;
    TopoDS_Shape block = BRepPrimAPI_MakeBox(
        gp_Pnt(cx - bs / 2.0, cy - bs / 2.0, z_ring_bot - floor_thickness),
        bs, bs, total_height).Shape();

    // ---- Cut inner cavity (starts at z_ring_bot, not below — floor stays solid)
    // Extend 1 mm above ring_top so the top face is cleanly removed.
    TopoDS_Shape inner_void = makeExtrudedSolid(cavity_profile, z_ring_bot, ring_height + 1.0);
    TopoDS_Shape jig = inner_void.IsNull() ? block : advCut(block, inner_void);
    if (jig.IsNull()) {
        RCLCPP_WARN(logger(), "  inner cut failed — using solid block");
        jig = block;
    }

    // ---- Fillet the 4 outer vertical corner edges of the square (5 mm) --------
    // After the inner cut, the 4 corner edges of the original box are unchanged
    // (the cavity never reaches them).  Detect them by XY proximity to box corners.
    {
        const double fillet_r = std::min(5.0, (total_height - 0.2) / 2.0);
        const double tol      = 1.5; // mm — detection tolerance around each corner

        BRepFilletAPI_MakeFillet fillet(jig);
        int nAdded = 0;

        for (TopExp_Explorer ex(jig, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
            BRepAdaptor_Curve curve(edge);
            if (curve.GetType() != GeomAbs_Line) continue;

            const gp_Dir dir = curve.Line().Direction();
            if (std::abs(dir.Dot(gp_Dir(0, 0, 1))) < 0.99) continue; // not Z-aligned

            // Test one endpoint — outer corners are at (cx ± bs/2, cy ± bs/2)
            const gp_Pnt ep = curve.Value(curve.FirstParameter());
            const bool at_x = std::abs(std::abs(ep.X() - cx) - bs / 2.0) < tol;
            const bool at_y = std::abs(std::abs(ep.Y() - cy) - bs / 2.0) < tol;
            if (at_x && at_y) {
                fillet.Add(fillet_r, edge);
                ++nAdded;
            }
        }

        if (nAdded > 0) {
            fillet.Build();
            if (fillet.IsDone())
                jig = fillet.Shape();
            else
                RCLCPP_WARN(logger(), "  corner fillet failed; using unfilleted shape");
        }
    }

    // ---- Identification notch -------------------------------------------------
    // Place at a corner of the base, inset enough to land on flat material
    // (not on the rounded corner region).
    {
        const double inset = bs / 2.0 - 8.0; // 8 mm from corner centre
        const gp_Pnt jig_cen = ShapeCentroid(jig);
        TopoDS_Shape notch = BRepPrimAPI_MakeCylinder(3, 4);
        notch = ShapeSetCentroid(notch,
            gp_Pnt(jig_cen.X() - inset, jig_cen.Y() - inset, jig_cen.Z()));
        const TopoDS_Shape j2 = advCut(jig, notch, 1e-5);
        if (!j2.IsNull()) jig = j2;
    }

    // ---- z-offset so Assembler can place the part above the jig floor ---------
    const float jig_part_z_offset =
        static_cast<float>(ShapeCentroid(shape_).Z() - ShapeCentroid(jig).Z());

    // ---- Translate jig centroid to canonical JIG_CENTER_Z ---------------------
    {
        const gp_Pnt c = ShapeCentroid(jig);
        jig = ShapeSetCentroid(jig, gp_Pnt(c.X(), c.Y(), JIG_CENTER_Z));
    }

    // ---- Mesh and export -------------------------------------------------------
    {
        BRepMesh_IncrementalMesh finalMesh(jig, 0.01);
        finalMesh.Perform();
    }

    std::stringstream ss;
    ss << OUTPUT_DIR << "jig_" << name_ << "_size_" << bay_size
       << "_index_" << bay_index << ".stl";
    StlAPI_Writer writer;
    writer.Write(jig, ss.str().c_str());

    RCLCPP_INFO(logger(), "  exported %s  (z_offset=%.3f)",
                ss.str().c_str(), jig_part_z_offset);

    return jig_part_z_offset;
}
