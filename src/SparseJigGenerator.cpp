/**
 * SparseJigGenerator
 *
 * Jig geometry:
 *   - Base: bay_size × bay_size × 5 mm, 5 mm rounded corners.
 *   - Part positioned so XY centroid aligns with bay XY centre; bottom of part
 *     is 5 mm above base top.
 *   - Vertical support cylinders (r = 1.5 mm) with slanted tops conforming to
 *     the part surface, generated from CandidateSupport contacts sampled at
 *     resolution R (default 0.1 mm) over every face and edge.
 *     Any cylinder whose shaft intersects the part body is discarded.
 *
 * Candidate contacts:
 *   - Face sample  → CandidateSupport with 1 Wrench; n_base = inward face normal.
 *   - Edge sample  → CandidateSupport with 2 Wrenches (one per adjacent face);
 *                    n_base = bisector of the two inward face normals.
 *
 * Cylinder top: a plane with normal = n_base, centred at (px, py, pz − 0.01 mm).
 */

#include "assembler/SparseJigGenerator.hpp"
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"
#include "assembler/ARMSConfig.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom_Plane.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
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
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <sstream>
#include <vector>

// ============================================================================
// Contact sample data structures
// ============================================================================

struct Wrench {
    gp_Pnt p;   // contact point relative to part centre of mass (mm)
    gp_Vec n;   // unit inward (internal) face normal at contact
};

struct CandidateSupport {
    std::vector<Wrench> wrenches;
    gp_Vec n_base;  // unit normal of the support cylinder's slanted top face
};

// ============================================================================
// Local geometry helpers  (sjg prefix avoids symbol collisions)
// ============================================================================

static TopoDS_Shape sjgMakeRoundedRectSolid(double cx, double cy, double z_bot,
                                             double side, double corner_r, double height)
{
    const double h = side * 0.5;
    BRepBuilderAPI_MakePolygon mkW;
    mkW.Add(gp_Pnt(cx - h, cy - h, z_bot));
    mkW.Add(gp_Pnt(cx + h, cy - h, z_bot));
    mkW.Add(gp_Pnt(cx + h, cy + h, z_bot));
    mkW.Add(gp_Pnt(cx - h, cy + h, z_bot));
    mkW.Close();

    auto fallback = [&]() {
        return BRepPrimAPI_MakeBox(gp_Pnt(cx - h, cy - h, z_bot), side, side, height).Shape();
    };
    if (!mkW.IsDone()) return fallback();

    Handle(Geom_Plane) hPlane = new Geom_Plane(gp_Pnt(cx, cy, z_bot), gp_Dir(0, 0, 1));
    BRepBuilderAPI_MakeFace mkF(hPlane, mkW.Wire(), Standard_True);
    if (!mkF.IsDone()) return fallback();

    BRepFilletAPI_MakeFillet2d mk2d(mkF.Face());
    {
        TopTools_IndexedMapOfShape vmap;
        TopExp::MapShapes(mkF.Face(), TopAbs_VERTEX, vmap);
        const double r = std::min(corner_r, h - 0.5);
        for (int i = 1; i <= vmap.Extent(); ++i)
            mk2d.AddFillet(TopoDS::Vertex(vmap(i)), r);
    }
    mk2d.Build();
    TopoDS_Shape rf = mk2d.IsDone() ? mk2d.Shape() : mkF.Face();

    BRepPrimAPI_MakePrism prism(rf, gp_Vec(0, 0, height));
    if (!prism.IsDone()) return fallback();
    return prism.Shape();
}

static TopoDS_Shape sjgFuse(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy = 1e-4)
{
    if (A.IsNull()) return B;
    if (B.IsNull()) return A;
    try {
        TopTools_ListOfShape args, tools;
        args.Append(A);
        tools.Append(B);
        BRepAlgoAPI_Fuse op;
        op.SetArguments(args);
        op.SetTools(tools);
        op.SetFuzzyValue(fuzzy);
        op.Build();
        if (!op.IsDone()) return A;
        const TopoDS_Shape r = op.Shape();
        return r.IsNull() ? A : r;
    } catch (...) {
        return A;
    }
}

static TopoDS_Shape sjgCut(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy = 1e-4)
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
    } catch (...) {
        return A;
    }
}

// Build a vertical cylinder from base_top up to (px_w, py_w, pz_w) with a slanted
// top face whose plane has normal = n_base and passes through (px_w, py_w, pz_w - 0.01).
// Returns a null shape when nz is too small to form a valid support cylinder.
static TopoDS_Shape sjgMakeSupportCylinder(double px_w, double py_w, double pz_w,
                                            double base_top, double cyl_r,
                                            const gp_Vec& n_base_unit)
{
    // Reject nearly horizontal support normals — they produce degenerate geometry.
    const double nz = n_base_unit.Z();
    if (nz < 0.05) return TopoDS_Shape();

    // Raw cylinder height: reach past the slant plane in the worst-case (nz near 0).
    const double slant_extra = cyl_r * std::sqrt(1.0 - nz * nz) / nz;
    const double cyl_h = pz_w - base_top + slant_extra + 5.0;
    if (cyl_h <= 0.0) return TopoDS_Shape();

    try {
        // 1. Tall upright cylinder.
        const gp_Ax2 ax(gp_Pnt(px_w, py_w, base_top), gp_Dir(0, 0, 1));
        TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(ax, cyl_r, cyl_h).Shape();
        if (cyl.IsNull()) return TopoDS_Shape();

        // 2. Cutting half-space: the region where n_base · (x − p_top) > 0.
        //    Centre of slanted face: p_top = (px_w, py_w, pz_w − 0.01).
        const gp_Pnt p_top(px_w, py_w, pz_w - 0.01);
        Handle(Geom_Plane) cut_plane =
            new Geom_Plane(p_top, gp_Dir(n_base_unit.X(), n_base_unit.Y(), n_base_unit.Z()));

        // Rectangular face on the plane — must be wider than the cylinder diameter.
        const double BIG = 150.0;
        BRepBuilderAPI_MakeFace mk_face(cut_plane->Pln(), -BIG, BIG, -BIG, BIG);
        if (!mk_face.IsDone()) return cyl;

        // Prism in n_base direction creates the solid half-space above the plane.
        BRepPrimAPI_MakePrism mk_prism(
            mk_face.Face(),
            gp_Vec(n_base_unit.X() * BIG, n_base_unit.Y() * BIG, n_base_unit.Z() * BIG));
        if (!mk_prism.IsDone()) return cyl;

        // 3. Remove the above-plane region → slanted-top cylinder.
        return sjgCut(cyl, mk_prism.Shape());

    } catch (...) {
        return TopoDS_Shape();
    }
}

// ============================================================================
// SparseJigGenerator::buildJigShape
// ============================================================================
TopoDS_Shape SparseJigGenerator::buildJigShape(float bay_size)
{
    RCLCPP_INFO(logger(), "SparseJigGenerator::buildJigShape: %s  bay=%.0f  R=%.3f mm",
                name_.c_str(), (double)bay_size, (double)sampling_resolution_);

    // -------------------------------------------------------------------------
    // Jig geometry constants
    // -------------------------------------------------------------------------
    constexpr double BASE_H   = 5.0;   // base height (mm)
    constexpr double PART_GAP = 5.0;   // clearance between base top and part bottom (mm)
    constexpr double CORNER_R = 5.0;   // rounded corner radius (mm)
    constexpr double CYL_R    = 1.5;   // support cylinder radius (mm) → 3 mm diameter

    // -------------------------------------------------------------------------
    // Part positioning
    // -------------------------------------------------------------------------
    const gp_Pnt part_cen = ShapeCentroid(shape_);
    const double cx  = part_cen.X();
    const double cy  = part_cen.Y();
    const double z_min = ShapeLowestPoint(shape_);

    const double base_bot = z_min - PART_GAP - BASE_H;
    const double base_top = z_min - PART_GAP;

    // Part centre of mass — wrench points are expressed relative to this.
    GProp_GProps vprops;
    BRepGProp::VolumeProperties(shape_, vprops);
    const gp_Pnt com = vprops.CentreOfMass();

    // -------------------------------------------------------------------------
    // Base plate
    // -------------------------------------------------------------------------
    const double bs = static_cast<double>(bay_size);
    TopoDS_Shape jig = sjgMakeRoundedRectSolid(cx, cy, base_bot, bs, CORNER_R, BASE_H);
    if (jig.IsNull()) {
        RCLCPP_ERROR(logger(), "SparseJigGenerator: base construction failed");
        return TopoDS_Shape();
    }

    // -------------------------------------------------------------------------
    // Edge → adjacent face map
    // -------------------------------------------------------------------------
    TopTools_IndexedDataMapOfShapeListOfShape e2f;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, e2f);

    const double R = static_cast<double>(sampling_resolution_);

    // -------------------------------------------------------------------------
    // Shaft collision check: would a vertical cylinder from base_top to pz
    // overlap the part?
    // -------------------------------------------------------------------------
    auto shaftCollides = [&](double px, double py, double pz) -> bool {
        const double shaft_h = pz - 0.01 - base_top;
        if (shaft_h <= 0.0) return true;
        try {
            const gp_Ax2 ax(gp_Pnt(px, py, base_top), gp_Dir(0, 0, 1));
            TopoDS_Shape shaft = BRepPrimAPI_MakeCylinder(ax, CYL_R, shaft_h).Shape();
            TopTools_ListOfShape args, tools;
            args.Append(shaft);
            tools.Append(shape_);
            BRepAlgoAPI_Common op;
            op.SetArguments(args);
            op.SetTools(tools);
            op.SetFuzzyValue(1e-4);
            op.Build();
            if (!op.IsDone() || op.Shape().IsNull()) return true;  // conservative
            return ShapeVolume(op.Shape()) > 1e-5;
        } catch (...) {
            return true;  // conservative: assume collision on error
        }
    };

    // Angle filter: keep only candidates whose n_base makes a 20°–70° angle with +Z.
    // cos(20°) ≈ 0.9397, cos(70°) ≈ 0.3420.
    constexpr double COS_UPPER = 0.9397;
    constexpr double COS_LOWER = 0.3420;

    // -------------------------------------------------------------------------
    // Collect CandidateSupports
    // -------------------------------------------------------------------------
    std::vector<CandidateSupport> candidates;
    int n_edge_cands = 0;
    int n_edge_angle_rejected = 0;

    int total_edges = 0;
    for (TopExp_Explorer ec(shape_, TopAbs_EDGE); ec.More(); ec.Next()) ++total_edges;
    RCLCPP_INFO(logger(), "  Sampling %d edges at R=%.2f mm", total_edges, R);

    // --- Edge sampling -------------------------------------------------------
    RCLCPP_INFO(logger(), "  Sampling edges...");
    TopTools_MapOfShape seen_edges;
    int edge_idx = 0;

    for (TopExp_Explorer exe(shape_, TopAbs_EDGE); exe.More(); exe.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(exe.Current());
        ++edge_idx;

        // Deduplicate: only process each geometric edge once.
        TopoDS_Edge fwd = edge;
        fwd.Orientation(TopAbs_FORWARD);
        if (!seen_edges.Add(fwd)) continue;

        const int fidx = e2f.FindIndex(edge);
        if (fidx < 1) continue;

        // Collect up to 2 adjacent faces.
        std::vector<TopoDS_Face> adj;
        for (TopTools_ListIteratorOfListOfShape it(e2f(fidx)); it.More(); it.Next()) {
            adj.push_back(TopoDS::Face(it.Value()));
            if (adj.size() == 2) break;
        }
        if (adj.size() != 2) continue;  // boundary / non-manifold edge

        try {
            BRepAdaptor_Curve curve(edge);
            const double t0  = curve.FirstParameter();
            const double t1  = curve.LastParameter();
            if (t1 - t0 < 1e-10) continue;

            // Approximate 3-D arc length by chord sampling.
            constexpr int ARC_DIV = 8;
            double arc_len = 0.0;
            {
                gp_Pnt prev = curve.Value(t0);
                for (int k = 1; k <= ARC_DIV; ++k) {
                    gp_Pnt cur = curve.Value(t0 + (t1 - t0) * k / ARC_DIV);
                    arc_len += prev.Distance(cur);
                    prev = cur;
                }
            }
            if (arc_len < 1e-6) continue;

            const int nsamp = std::max(2, static_cast<int>(std::ceil(arc_len / R)));

            for (int is = 0; is <= nsamp; ++is) {
                const double t   = t0 + (t1 - t0) * is / nsamp;
                const gp_Pnt p3d = curve.Value(t);

                // Outward normals via the tested MeshFunctions helper — consistent
                // across all face types and avoids parameter-range ambiguity from
                // BRep_Tool::CurveOnSurface on REVERSED edges.
                const gp_Dir od0 = outwardFaceNormal(adj[0]);
                const gp_Dir od1 = outwardFaceNormal(adj[1]);

                // Inward normals.
                const gp_Vec in0(-od0.X(), -od0.Y(), -od0.Z());
                const gp_Vec in1(-od1.X(), -od1.Y(), -od1.Z());

                // n_base = bisector of the two inward normals.
                gp_Vec bis = in0 + in1;
                if (bis.SquareMagnitude() < 1e-10) continue;
                bis.Normalize();

                // Angle filter: discard if bisector angle with Z < 20° or > 70°.
                if (bis.Z() < COS_LOWER || bis.Z() > COS_UPPER) {
                    ++n_edge_angle_rejected;
                    continue;
                }

                const gp_Pnt p_rel(p3d.X() - com.X(),
                                    p3d.Y() - com.Y(),
                                    p3d.Z() - com.Z());

                Wrench w0, w1;
                w0.p = p_rel;  w0.n = in0;
                w1.p = p_rel;  w1.n = in1;

                CandidateSupport cs;
                cs.wrenches = {w0, w1};
                cs.n_base   = bis;
                candidates.push_back(std::move(cs));
                ++n_edge_cands;
            }
        } catch (...) {}
    }
    RCLCPP_INFO(logger(), "  Edge sampling done: %d candidates (%d angle-rejected)",
                n_edge_cands, n_edge_angle_rejected);
    RCLCPP_INFO(logger(), "  Total candidates: %zu", candidates.size());

    // =========================================================================
    // Pass 1 — Collision check: discard any candidate whose shaft intersects the part.
    // =========================================================================
    {
        std::vector<CandidateSupport> survivors;
        survivors.reserve(candidates.size());
        int n_coll = 0;
        for (auto& cs : candidates) {
            const Wrench& w = cs.wrenches[0];
            const double px_w = com.X() + w.p.X();
            const double py_w = com.Y() + w.p.Y();
            const double pz_w = com.Z() + w.p.Z();
            if (shaftCollides(px_w, py_w, pz_w)) { ++n_coll; continue; }
            survivors.push_back(std::move(cs));
        }
        RCLCPP_INFO(logger(), "  After collision check: %zu survivors (%d discarded)",
                    survivors.size(), n_coll);
        candidates = std::move(survivors);
    }

    // =========================================================================
    // Pass 2 — Wrench binning: keep at most one candidate per 6D wrench-space bin.
    //
    // Each wrench maps to a 6-vector [n ; p×n].  Bin = 6-bit sign pattern.
    // Per bin we retain the candidate with the highest wrench 6D norm.
    // =========================================================================
    {
        constexpr int N_BINS = 64;  // 2^6
        std::array<double, N_BINS> best_norm;
        std::array<int,    N_BINS> best_ci;
        best_norm.fill(-1.0);
        best_ci.fill(-1);

        auto wrenchBin = [](const Wrench& w) -> int {
            const gp_Vec pv(w.p.X(), w.p.Y(), w.p.Z());
            const gp_Vec m = pv.Crossed(w.n);
            const double w6[6] = { w.n.X(), w.n.Y(), w.n.Z(),
                                    m.X(),   m.Y(),   m.Z() };
            int bin = 0;
            for (int k = 0; k < 6; ++k)
                bin = (bin << 1) | (w6[k] > 0.0 ? 1 : 0);
            return bin;
        };
        auto wrenchNorm = [](const Wrench& w) -> double {
            const gp_Vec pv(w.p.X(), w.p.Y(), w.p.Z());
            const gp_Vec m = pv.Crossed(w.n);
            return std::sqrt(w.n.SquareMagnitude() + m.SquareMagnitude());
        };

        for (int ci = 0; ci < static_cast<int>(candidates.size()); ++ci)
            for (const auto& wr : candidates[ci].wrenches) {
                const int    b  = wrenchBin(wr);
                const double nm = wrenchNorm(wr);
                if (nm > best_norm[b]) { best_norm[b] = nm; best_ci[b] = ci; }
            }

        std::set<int> selected;
        for (int b = 0; b < N_BINS; ++b)
            if (best_ci[b] >= 0) selected.insert(best_ci[b]);

        std::vector<CandidateSupport> pruned;
        pruned.reserve(selected.size());
        for (int ci : selected) pruned.push_back(std::move(candidates[ci]));
        candidates = std::move(pruned);

        RCLCPP_INFO(logger(), "  After wrench binning: %zu candidates (max %d bins)",
                    candidates.size(), N_BINS);
    }

    // =========================================================================
    // Pass 3 — Build and fuse cylinders.
    // =========================================================================
    std::vector<CandidateSupport> added_supports;
    added_supports.reserve(candidates.size());

    for (auto& cs : candidates) {
        const Wrench& w = cs.wrenches[0];
        const double px_w = com.X() + w.p.X();
        const double py_w = com.Y() + w.p.Y();
        const double pz_w = com.Z() + w.p.Z();

        TopoDS_Shape cyl = sjgMakeSupportCylinder(
            px_w, py_w, pz_w, base_top, CYL_R, cs.n_base);
        if (cyl.IsNull()) continue;

        jig = sjgFuse(jig, cyl);
        added_supports.push_back(cs);
    }

    RCLCPP_INFO(logger(), "  Done: %zu cylinders added",  added_supports.size());

    // =========================================================================
    // Report — list every selected support with its wrenches and geometry.
    // =========================================================================
    RCLCPP_INFO(logger(), "=== Jig report: %s  (%zu supports) ===",
                name_.c_str(), added_supports.size());
    for (int i = 0; i < static_cast<int>(added_supports.size()); ++i) {
        const auto& cs = added_supports[i];
        RCLCPP_INFO(logger(),
            "  [%d]  n_base=(%.4f, %.4f, %.4f)",
            i, cs.n_base.X(), cs.n_base.Y(), cs.n_base.Z());
        for (int j = 0; j < static_cast<int>(cs.wrenches.size()); ++j) {
            const auto& wr = cs.wrenches[j];
            const double wx = com.X() + wr.p.X();
            const double wy = com.Y() + wr.p.Y();
            const double wz = com.Z() + wr.p.Z();
            RCLCPP_INFO(logger(),
                "       wrench %d:  pos=(%.3f, %.3f, %.3f)  n=(%.4f, %.4f, %.4f)"
                "  p_rel=(%.3f, %.3f, %.3f)",
                j, wx, wy, wz,
                wr.n.X(), wr.n.Y(), wr.n.Z(),
                wr.p.X(), wr.p.Y(), wr.p.Z());
        }
    }

    return jig;
}

// ============================================================================
// SparseJigGenerator::createJig
// ============================================================================
float SparseJigGenerator::createJig(float bay_size, int bay_index,
                                     const std::string& output_dir)
{
    TopoDS_Shape jig = buildJigShape(bay_size);
    if (jig.IsNull()) {
        RCLCPP_ERROR(logger(), "SparseJigGenerator::createJig: buildJigShape returned null");
        return 0.0f;
    }

    // The Assembler sets the part centroid to JIG_CENTER_Z before calling us.
    // buildJigShape built the jig with base_top = part_bottom - 5 mm (PART_GAP).
    // We want the exported STL to have base_top at JIG_CENTER_Z, so the simulation
    // can load it at world origin and get the correct part-tray height.
    constexpr double PART_GAP = 5.0;
    const double part_lo   = ShapeLowestPoint(shape_);
    const double base_top  = part_lo - PART_GAP;          // Z of base top in build frame
    const double delta_z   = JIG_CENTER_Z - base_top;     // shift needed

    const gp_Pnt jig_cen = ShapeCentroid(jig);
    jig = ShapeSetCentroid(jig, gp_Pnt(jig_cen.X(), jig_cen.Y(), jig_cen.Z() + delta_z));

    // Part centroid world Z: base_top at JIG_CENTER_Z, part bottom 5 mm above that,
    // part centroid a further centroid_h above the part bottom.
    const double centroid_h = ShapeCentroid(shape_).Z() - part_lo;
    const float  z_offset   = static_cast<float>(JIG_CENTER_Z + PART_GAP + centroid_h);

    BRepMesh_IncrementalMesh(jig, 0.1).Perform();

    std::ostringstream oss;
    oss << output_dir << "jig_" << name_
        << "_size_" << bay_size
        << "_index_" << bay_index << ".stl";
    StlAPI_Writer().Write(jig, oss.str().c_str());

    RCLCPP_INFO(logger(), "  exported %s  (z_offset=%.3f)", oss.str().c_str(), z_offset);
    return z_offset;
}
