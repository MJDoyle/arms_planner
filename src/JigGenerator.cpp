/**
 * JigGenerator
 *
 * Jig geometry:
 *   - Square base: bay_size × bay_size × 5 mm, 5 mm rounded corners.
 *   - Part sits 5 mm above the base top.
 *   - 3 mm-diameter support cylinders under every horizontal (XY-plane) edge
 *     of the part.  Each cylinder spans from the base top to 1 mm above the
 *     edge midpoint; the top is chamfered like TestCradleGenerator.
 *     Cylinders whose shafts intersect the part body are discarded.
 */

#include "assembler/JigGenerator.hpp"
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"
#include "assembler/ARMSConfig.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
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
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>

// ---------------------------------------------------------------------------
// Rounded-rectangle base solid (side × side × height, corner radius r).
// ---------------------------------------------------------------------------
static TopoDS_Shape jgMakeRoundedRectSolid(double cx, double cy, double z_bot,
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
static TopoDS_Shape jgFuse(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy = 1e-4)
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

static TopoDS_Shape jgCut(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy = 1e-4)
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
// JigGenerator::buildJigShape  — geometry only, no STL / no JIG_CENTER_Z move
// ===========================================================================
TopoDS_Shape JigGenerator::buildJigShape(float bay_size)
{
    const double z_min = ShapeLowestPoint(shape_);
    const gp_Pnt part_cen = ShapeCentroid(shape_);
    const double cx = part_cen.X(), cy = part_cen.Y();
    const double bs = (double)bay_size;

    constexpr double FLOOR_H   = 5.0;
    constexpr double GAP       = 5.0;
    constexpr double CORNER_R  = 5.0;
    constexpr double CYL_R     = 1.5;   // 3 mm diameter
    constexpr double CYL_ABOVE = 1.0;   // mm above edge midpoint
    constexpr double TOL_Z     = 0.5;   // max z-spread for a "horizontal" edge
    constexpr double MIN_TILT  = 30.0;  // min face angle from Z to apply chamfer (°)

    const double base_bot = z_min - GAP - FLOOR_H;
    const double base_top = z_min - GAP;

    // ---- Base plate ---------------------------------------------------------
    TopoDS_Shape jig = jgMakeRoundedRectSolid(cx, cy, base_bot, bs, CORNER_R, FLOOR_H);
    if (jig.IsNull()) { RCLCPP_ERROR(logger(), "JigGenerator: base failed"); return TopoDS_Shape(); }

    // ---- Edge-to-face adjacency map -----------------------------------------
    TopTools_IndexedDataMapOfShapeListOfShape e2f;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, e2f);

    // ---- Shared candidate struct --------------------------------------------
    struct Candidate {
        gp_Pnt mp;
        gp_Vec tang;
        gp_Vec face_normal;
        double face_angle_deg;
        double cyl_h;
        bool   from_face;   // true = curved-face source, false = horizontal edge
    };
    std::vector<Candidate> candidates;

    int n_horiz = 0, n_face_pts = 0, n_collide = 0;

    // ---- Helper: collision check (shaft from base_top to z_c - 0.1 mm) -----
    auto shaftCollides = [&](double px, double py, double z_c) -> bool {
        const double test_h = z_c - 0.1 - base_top;
        if (test_h <= 0.0) return false;
        try {
            const gp_Ax2 ax(gp_Pnt(px, py, base_top), gp_Dir(0, 0, 1));
            TopoDS_Shape tc = BRepPrimAPI_MakeCylinder(ax, CYL_R, test_h).Shape();
            TopTools_ListOfShape args, tools;
            args.Append(tc); tools.Append(shape_);
            BRepAlgoAPI_Common op;
            op.SetArguments(args); op.SetTools(tools);
            op.SetFuzzyValue(1e-4); op.Build();
            return op.IsDone() && !op.Shape().IsNull() && ShapeVolume(op.Shape()) > 0.1;
        } catch (...) { return false; }
    };

    // ---- Pass 1a: horizontal (XY-plane) edges -------------------------------
    TopTools_MapOfShape seen;

    for (TopExp_Explorer ex(shape_, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
        TopoDS_Edge fwd = edge; fwd.Orientation(TopAbs_FORWARD);
        if (!seen.Add(fwd)) continue;

        try {
            BRepAdaptor_Curve adaptor(edge);
            const double t0 = adaptor.FirstParameter();
            const double t1 = adaptor.LastParameter();
            if (t1 - t0 < 1e-10) continue;

            double z_lo = 1e30, z_hi = -1e30;
            for (int s = 0; s <= 4; ++s) {
                const double z = adaptor.Value(t0 + (t1 - t0) * s / 4.0).Z();
                z_lo = std::min(z_lo, z); z_hi = std::max(z_hi, z);
            }
            if (z_hi - z_lo > TOL_Z) continue;
            ++n_horiz;

            gp_Pnt mp; gp_Vec tang;
            adaptor.D1((t0 + t1) / 2.0, mp, tang);
            if (tang.Magnitude() < 1e-10) continue;
            tang.Normalize();

            const int fidx = e2f.FindIndex(edge);
            if (fidx < 1) continue;

            double best_lat = -1.0;
            gp_Vec face_normal;
            double face_angle_deg = 0.0;
            for (TopTools_ListIteratorOfListOfShape it(e2f(fidx)); it.More(); it.Next()) {
                const TopoDS_Face& f = TopoDS::Face(it.Value());
                try {
                    gp_Dir n = outwardFaceNormal(f);
                    const double lat = std::sqrt(n.X()*n.X() + n.Y()*n.Y());
                    if (lat > best_lat) {
                        best_lat       = lat;
                        face_normal    = gp_Vec(n);
                        face_angle_deg = std::acos(std::min(1.0, std::abs(n.Z()))) * 180.0 / M_PI;
                    }
                } catch (...) {}
            }

            const double cyl_h = (mp.Z() + CYL_ABOVE) - base_top;
            if (cyl_h <= 0.0) continue;

            if (shaftCollides(mp.X(), mp.Y(), mp.Z())) { ++n_collide; continue; }

            candidates.push_back({mp, tang, face_normal, face_angle_deg, cyl_h, false});
        } catch (...) {}
    }

    // ---- Pass 1b: curved faces – points where normal is 45° below horizontal
    // The outward normal at a 45°-down point has nZ = −sin(45°) = −√2/2 ≈ −0.707.
    // Use the horizontal projection of that normal as the chamfer tilt direction.
    constexpr double TARGET_NZ = -M_SQRT1_2;   // −√2/2
    constexpr double TOL_NZ    = 0.10;          // accept nZ in [−0.807, −0.607]
    constexpr int    NSAMP     = 5;             // samples per UV axis per face

    for (TopExp_Explorer exf(shape_, TopAbs_FACE); exf.More(); exf.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exf.Current());
        try {
            BRepAdaptor_Surface surf(face, Standard_True);
            if (surf.GetType() == GeomAbs_Plane) continue;   // skip flat faces

            const double u0 = surf.FirstUParameter(),  u1 = surf.LastUParameter();
            const double v0 = surf.FirstVParameter(),  v1 = surf.LastVParameter();
            if (!std::isfinite(u0) || !std::isfinite(u1) ||
                !std::isfinite(v0) || !std::isfinite(v1)) continue;
            if (u1 - u0 < 1e-10 || v1 - v0 < 1e-10) continue;

            for (int iu = 0; iu <= NSAMP; ++iu) {
                for (int iv = 0; iv <= NSAMP; ++iv) {
                    const double u = u0 + (u1 - u0) * iu / NSAMP;
                    const double v = v0 + (v1 - v0) * iv / NSAMP;
                    try {
                        gp_Pnt p; gp_Vec d1u, d1v;
                        surf.D1(u, v, p, d1u, d1v);

                        gp_Vec n = d1u.Crossed(d1v);
                        if (n.Magnitude() < 1e-10) continue;
                        n.Normalize();
                        if (face.Orientation() == TopAbs_REVERSED) n.Reverse();

                        if (std::abs(n.Z() - TARGET_NZ) > TOL_NZ) continue;
                        ++n_face_pts;

                        // Tangent = perpendicular to horizontal projection of normal
                        gp_Vec nxy(n.X(), n.Y(), 0.0);
                        if (nxy.Magnitude() < 1e-10) continue;
                        nxy.Normalize();
                        const gp_Vec tang(-nxy.Y(), nxy.X(), 0.0);

                        const double face_angle_deg =
                            std::acos(std::min(1.0, std::abs(n.Z()))) * 180.0 / M_PI;
                        const double cyl_h = (p.Z() + CYL_ABOVE) - base_top;
                        if (cyl_h <= 0.0) continue;

                        if (shaftCollides(p.X(), p.Y(), p.Z())) { ++n_collide; continue; }

                        candidates.push_back({p, tang, n, face_angle_deg, cyl_h, true});
                    } catch (...) {}
                }
            }
        } catch (...) {}
    }

    RCLCPP_INFO(logger(),
        "  candidates: %d from edges (%d horiz), %d from curved faces  (%d collide)",
        (int)std::count_if(candidates.begin(), candidates.end(),
                           [](const Candidate& c){ return !c.from_face; }),
        n_horiz,
        (int)std::count_if(candidates.begin(), candidates.end(),
                           [](const Candidate& c){ return  c.from_face; }),
        n_collide);

    // ---- Pass 2: select TWO candidates per cardinal direction (8 total) ------
    // For each direction the pool is all candidates with positive principal value
    // (i.e. on the correct side of the CoM).  From that pool we pick the pair
    // that maximises perpendicular spread: the candidate with the highest
    // perpendicular coordinate and the one with the lowest.
    // If fewer than 2 candidates have a positive principal value the pool is
    // widened to the top half of all candidates ranked by principal value.

    // Returns {hi_perp, lo_perp} — either may be nullptr.
    auto selectTwo = [&](int dim, bool want_max)
        -> std::pair<const Candidate*, const Candidate*>
    {
        // Signed principal distance from CoM along this axis
        auto pri = [&](const Candidate& c) -> double {
            const double dp = (dim == 0) ? (c.mp.X() - cx) : (c.mp.Y() - cy);
            return want_max ? dp : -dp;
        };
        // Perpendicular coordinate (raw, not distance)
        auto perp_coord = [&](const Candidate& c) -> double {
            return (dim == 0) ? c.mp.Y() : c.mp.X();
        };

        // Primary pool: candidates on the correct side of the CoM
        std::vector<const Candidate*> pool;
        for (const auto& c : candidates)
            if (pri(c) >= 0.0) pool.push_back(&c);

        // Fallback: not enough candidates on this side — take top half by principal
        if (pool.size() < 2) {
            pool.clear();
            for (const auto& c : candidates) pool.push_back(&c);
            std::sort(pool.begin(), pool.end(),
                      [&](const Candidate* a, const Candidate* b){ return pri(*a) > pri(*b); });
            const size_t keep = std::max(size_t(2), (pool.size() + 1) / 2);
            if (pool.size() > keep) pool.resize(keep);
        }

        if (pool.empty()) return {nullptr, nullptr};
        if (pool.size() == 1) return {pool[0], nullptr};

        // Find the pair that spans the widest perpendicular range
        const Candidate* hi = *std::max_element(pool.begin(), pool.end(),
            [&](const Candidate* a, const Candidate* b){
                return perp_coord(*a) < perp_coord(*b); });
        const Candidate* lo = *std::min_element(pool.begin(), pool.end(),
            [&](const Candidate* a, const Candidate* b){
                return perp_coord(*a) < perp_coord(*b); });

        if (hi == lo) return {hi, nullptr};
        return {hi, lo};
    };

    // Two slots per direction; nullptr means "no candidate available"
    const Candidate* sel[8] = {};
    const char* labels[8] = {
        "+x[0]", "+x[1]",
        "-x[0]", "-x[1]",
        "+y[0]", "+y[1]",
        "-y[0]", "-y[1]",
    };
    {
        auto [a, b] = selectTwo(0, true);   sel[0]=a; sel[1]=b;
        auto [c, d] = selectTwo(0, false);  sel[2]=c; sel[3]=d;
        auto [e, f] = selectTwo(1, true);   sel[4]=e; sel[5]=f;
        auto [g, h] = selectTwo(1, false);  sel[6]=g; sel[7]=h;
    }

    // ---- Pass 3: build cylinders for the eight selected candidates ----------
    int n_kept = 0;
    for (int i = 0; i < 8; ++i) {
        const Candidate* c = sel[i];
        if (!c) { RCLCPP_WARN(logger(), "  %s: no candidate", labels[i]); continue; }

        try {
            const gp_Ax2 axis(gp_Pnt(c->mp.X(), c->mp.Y(), base_top), gp_Dir(0, 0, 1));
            TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(axis, CYL_R, c->cyl_h + 30.0).Shape();

            const double tilt_deg = c->face_angle_deg / 2.0;
            const double tilt_rad = tilt_deg * M_PI / 180.0;
            if (tilt_rad > 1e-6 && c->face_normal.Magnitude() > 1e-10
                && c->face_angle_deg >= MIN_TILT)
            {
                try {
                    gp_Dir tilt_axis(c->tang);
                    gp_Vec cross_Z_n = gp_Vec(0, 0, 1).Crossed(c->face_normal);
                    const double align = (cross_Z_n.Magnitude() > 1e-10)
                                         ? cross_Z_n.Dot(gp_Vec(tilt_axis)) : 0.0;
                    const double rot_sign = (align >= 0) ? -1.0 : 1.0;

                    constexpr double SLAB = 60.0;
                    TopoDS_Shape cutter = BRepPrimAPI_MakeBox(
                        gp_Pnt(-SLAB, -SLAB, 0.0), 2*SLAB, 2*SLAB, SLAB).Shape();
                    gp_Trsf rot, trans;
                    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), tilt_axis), rot_sign * tilt_rad);
                    trans.SetTranslation(gp_Vec(c->mp.X(), c->mp.Y(), base_top + c->cyl_h));
                    BRepBuilderAPI_Transform mkT(cutter, trans * rot, Standard_True);
                    if (mkT.IsDone()) {
                        TopoDS_Shape cut = jgCut(cyl, mkT.Shape());
                        if (!cut.IsNull()) cyl = cut;
                    }
                } catch (...) {
                    RCLCPP_WARN(logger(), "  %s: chamfer failed, flat top", labels[i]);
                }
            }

            jig = jgFuse(jig, cyl);
            ++n_kept;
            // Perpendicular offset from CoM for this direction
            const double perp_off = (i < 4) ? (c->mp.Y() - cy) : (c->mp.X() - cx);
            RCLCPP_INFO(logger(),
                "  %s peg @ (%.2f,%.2f,%.2f)  h=%.2f  tilt=%.1f°  perp_off=%.2f  src=%s",
                labels[i], c->mp.X(), c->mp.Y(), c->mp.Z(),
                c->cyl_h, tilt_deg, perp_off, c->from_face ? "face" : "edge");
        } catch (...) {
            RCLCPP_WARN(logger(), "  %s: cylinder construction failed", labels[i]);
        }
    }

    return jig;
}

// ===========================================================================
// JigGenerator::createJig  — build geometry, translate to JIG_CENTER_Z, export
// ===========================================================================
float JigGenerator::createJig(float bay_size, int bay_index, const std::string& output_dir)
{
    RCLCPP_INFO(logger(), "JigGenerator: %s  bay=%g #%d",
                name_.c_str(), (double)bay_size, bay_index);

    TopoDS_Shape jig = buildJigShape(bay_size);

    const double jig_lo = ShapeLowestPoint(jig);
    const float jig_part_z_offset =
        static_cast<float>(ShapeCentroid(shape_).Z() - jig_lo);

    {
        const gp_Pnt c = ShapeCentroid(jig);
        jig = ShapeSetCentroid(jig, gp_Pnt(c.X(), c.Y(), c.Z() - jig_lo));
    }

    BRepMesh_IncrementalMesh(jig, 0.01).Perform();

    std::stringstream ss;
    ss << output_dir << "jig_" << name_ << "_size_" << bay_size
       << "_index_" << bay_index << ".stl";
    StlAPI_Writer().Write(jig, ss.str().c_str());

    RCLCPP_INFO(logger(), "  exported %s  (z_offset=%.3f)", ss.str().c_str(), jig_part_z_offset);
    return jig_part_z_offset;
}
