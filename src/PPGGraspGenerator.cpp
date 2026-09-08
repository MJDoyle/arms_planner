#include "assembler/PPGGraspGenerator.hpp"

#include "assembler/Assembler.hpp"   // GripperConfig
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"
#include "assembler/Part.hpp"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Parallel-plate grasp synthesis.
//
// The gripper rotates about z only, so the jaw axis is always horizontal and the
// approach is always top-down.  That reduces a grasp to four numbers — angle,
// jaw height, and the two contact positions along the axis — and lets the search
// work directly in the gripper's own configuration space instead of proposing
// 3D contact pairs and discarding those it cannot reach.
//
// For a horizontal axis u, the jaws close onto whatever protrudes furthest along
// ±u within the jaw's vertical span.  So the contact is the support of the part
// over that z-band, and the shape of the supporting set tells us what kind of
// contact it is:
//
//     spread in z AND across the jaw  ->  flat vertical face
//     spread in z only                ->  vertical line (a cylinder side)
//     spread across the jaw only      ->  horizontal line (a sloped/chamfered wall)
//     spread in neither               ->  point (a corner)
//
// Verticality is not required — a tilted face grips provided the closing
// direction stays inside the friction cone, i.e. slope <= atan(mu).  What is
// required is that we measure the slope in 3D; a plan-view cross-section cannot
// see it, and a jaw pressed on too steep a face cams the part out.
// ---------------------------------------------------------------------------

namespace {

constexpr double ANGLE_STEP_DEG = 7.5;   // jaw axis sweep
constexpr double Z_STEP_MM      = 1.0;   // jaw height sweep
constexpr double CONTACT_TOL_MM = 0.4;   // how close to the support plane still counts as touching

// Above these fractions of the jaw face, a contact is *described* as broad or
// vertical.  They label the contact for reporting; they do not gate it.
constexpr double FLAT_FRAC_Z = 0.60;
constexpr double FLAT_FRAC_W = 0.50;

// The jaw does not have to be fully covered — it may hang above a short part, and
// often only part of its face touches.  All that is genuinely required is enough
// contact to be a contact at all.
// Deliberately small: this only rules out a degenerate knife-edge touch.  How
// much of the jaw is covered is a matter of degree, handled by the score, so a
// thin plate gripped across 1 mm of its edge is admissible — just ranked below a
// grip that gets more of the jaw onto the part.
constexpr double MIN_CONTACT_MM = 0.5;

// Relative weight of the two competing properties.  Slope dominates because
// exceeding the friction cone loses the part outright, whereas a small contact
// patch only lets it pivot in the jaws — so a narrow grip on a vertical edge is
// preferred to a broad one on a slope.
constexpr double W_SLOPE   = 0.65;
constexpr double W_CONTACT = 0.35;

// Surface points of a shape, in world mm.
//
// Triangulation vertices alone are not enough.  A developable surface — the side
// of a cylinder being the case that matters here — tessellates into quads that
// span its whole height however fine the chord tolerance, because deflection
// controls subdivision around the curve, not along it.  Sampling only vertices
// then leaves a 14 mm cylinder with points at z = 0 and z = 14 and nothing
// between, so any jaw band strictly inside it looks empty and the contact is
// misclassified.  Each triangle is therefore sampled across its interior, at a
// density set by its own size.
struct SurfPt { gp_Pnt p; gp_Dir n; };   // point and its outward surface normal

std::vector<SurfPt> surfacePoints(const TopoDS_Shape& shape, double deflection)
{
    constexpr double TARGET_SPACING_MM = 1.0;
    constexpr int    MAX_SUBDIV        = 10;

    std::vector<SurfPt> pts;

    BRepMesh_IncrementalMesh mesher(shape, deflection);
    mesher.Perform();

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        const gp_Trsf trsf = loc.IsIdentity() ? gp_Trsf() : loc.Transformation();

        for (int t = 1; t <= tri->NbTriangles(); ++t)
        {
            int i1, i2, i3;
            tri->Triangle(t).Get(i1, i2, i3);
            const gp_Pnt a = tri->Node(i1).Transformed(trsf);
            const gp_Pnt b = tri->Node(i2).Transformed(trsf);
            const gp_Pnt c = tri->Node(i3).Transformed(trsf);

            // Outward normal of this facet.  This is what the friction-cone test
            // needs: the support geometry alone cannot tell a vertical edge from a
            // tilted face, since both touch the jaw along a thin strip.
            gp_Vec nv = gp_Vec(a, b).Crossed(gp_Vec(a, c));
            if (nv.Magnitude() < 1e-12) continue;
            if (face.Orientation() == TopAbs_REVERSED) nv.Reverse();
            const gp_Dir nrm(nv);

            const double longest = std::max({a.Distance(b), b.Distance(c), c.Distance(a)});
            const int n = std::clamp(static_cast<int>(std::ceil(longest / TARGET_SPACING_MM)),
                                     1, MAX_SUBDIV);

            for (int i = 0; i <= n; ++i)
                for (int j = 0; i + j <= n; ++j)
                {
                    const double wa = double(n - i - j) / n;
                    const double wb = double(i) / n;
                    const double wc = double(j) / n;
                    pts.push_back({ gp_Pnt(wa * a.X() + wb * b.X() + wc * c.X(),
                                            wa * a.Y() + wb * b.Y() + wc * c.Y(),
                                            wa * a.Z() + wb * b.Z() + wc * c.Z()), nrm });
                }
        }
    }
    return pts;
}

struct Contact {
    double  extent_a = 0.0;    // position along the grasp axis
    double  spread_z = 0.0;    // vertical size of the touching region
    double  spread_w = 0.0;    // size across the jaw face
    double  slope    = 0.0;    // radians from vertical
    bool    found    = false;
};

// Where one jaw meets the part, given the axis, the z-band, and which side.
// `sign` = +1 for the jaw closing from +u, -1 from -u.
Contact findContact(const std::vector<std::pair<double, SurfPt>>& band,  // (a = p.u, sample)
                    const gp_Dir& u, const gp_Dir& perp, int sign)
{
    Contact c;
    if (band.empty()) return c;

    // The jaw stops at the furthest-protruding point along its closing direction.
    double extreme = sign > 0 ? -1e18 : 1e18;
    for (auto const& [a, sp] : band)
        extreme = sign > 0 ? std::max(extreme, a) : std::min(extreme, a);

    // Everything within tolerance of that plane is in contact.  The slope that
    // matters is the angle between the jaw's closing direction and the actual
    // surface normal there — that is the friction-cone condition, and it is the
    // one thing the support geometry cannot tell us: a vertical edge and a tilted
    // face both touch along a thin strip, but only the tilted one cams the part
    // out of the jaws.
    const gp_Dir close = sign > 0 ? u.Reversed() : u;   // direction the jaw pushes

    const gp_Dir face_dir = close.Reversed();   // the direction a contacting face must look

    double z_min = 1e18, z_max = -1e18, w_min = 1e18, w_max = -1e18;
    double best_angle = M_PI;
    int n = 0, n_facing = 0;

    for (auto const& [a, sp] : band)
    {
        if (std::abs(a - extreme) > CONTACT_TOL_MM) continue;
        const double w = sp.p.X() * perp.X() + sp.p.Y() * perp.Y();
        z_min = std::min(z_min, sp.p.Z());  z_max = std::max(z_max, sp.p.Z());
        w_min = std::min(w_min, w);         w_max = std::max(w_max, w);
        ++n;

        // Only surfaces that actually look towards the jaw can bear on it.  The
        // tolerance band also catches the rims of perpendicular faces — the top of
        // a box, say — whose normals are square to the closing direction and would
        // otherwise make a flat vertical face read as steeply sloped.
        if (sp.n.Dot(face_dir) <= 1e-6) continue;
        ++n_facing;

        // The jaw is rigid and flat, so it bears on the best-aligned facet at the
        // support plane; neighbouring material is fractionally recessed behind it.
        best_angle = std::min(best_angle, sp.n.Angle(face_dir));
    }
    if (n == 0 || n_facing == 0) return c;

    c.slope = best_angle;

    c.extent_a = extreme;
    c.spread_z = z_max - z_min;
    c.spread_w = w_max - w_min;
    c.found    = true;
    return c;
}

// Feasible only if both contacts sit inside the friction cone and actually touch.
bool feasible(const Contact& a, const Contact& b, double mu_limit)
{
    if (!a.found || !b.found) return false;
    if (a.slope > mu_limit || b.slope > mu_limit) return false;
    return std::min(a.spread_z, b.spread_z) >= MIN_CONTACT_MM;
}

// A descriptive label only — see the header.
PPGContactClass classify(const Contact& a, const Contact& b, const GripperConfig& cfg)
{
    const double flat_z = FLAT_FRAC_Z * cfg.jaw_height_mm;
    const double flat_w = FLAT_FRAC_W * cfg.jaw_width_mm;
    const bool vert = (a.spread_z >= flat_z) && (b.spread_z >= flat_z);
    const bool wide = (a.spread_w >= flat_w) && (b.spread_w >= flat_w);
    if (vert && wide) return PPGContactClass::VerticalFace;
    if (vert)         return PPGContactClass::VerticalLine;
    if (wide)         return PPGContactClass::SlopedFace;
    return PPGContactClass::PointContact;
}

// Continuous grasp quality in [0, 1], taken from the worse of the two contacts
// since a grip is only as good as its weaker side.
//
// Slope is scored as remaining margin inside the friction cone, contact as the
// fraction of the jaw face that touches.  Normalising contact against the jaw
// rather than against what the part can offer is deliberate: a short part really
// is less securely held in a tall jaw, and the score should say so.
double quality(const Contact& a, const Contact& b,
               const GripperConfig& cfg, double mu_limit)
{
    const double worst_slope = std::max(a.slope, b.slope);
    const double slope_term  = std::clamp(1.0 - worst_slope / std::max(1e-6, mu_limit), 0.0, 1.0);

    const double jaw_area = cfg.jaw_height_mm * cfg.jaw_width_mm;
    const double area_a = std::min(a.spread_z, cfg.jaw_height_mm) * std::min(a.spread_w, cfg.jaw_width_mm);
    const double area_b = std::min(b.spread_z, cfg.jaw_height_mm) * std::min(b.spread_w, cfg.jaw_width_mm);
    const double contact_term = std::clamp(std::min(area_a, area_b) / std::max(1e-6, jaw_area), 0.0, 1.0);

    return W_SLOPE * slope_term + W_CONTACT * contact_term;
}

}  // namespace

// ---------------------------------------------------------------------------

PartPPGCandidates PPGGraspGenerator::precompute(std::shared_ptr<Part> part,
                                                const GripperConfig&  cfg)
{
    PartPPGCandidates result;
    if (!part || !part->getShape()) return result;

    const TopoDS_Shape& shape = *part->getShape();
    const auto pts = surfacePoints(shape, 0.2);
    if (pts.size() < 4) return result;

    const gp_Pnt com = ShapeCenterOfMass(shape);
    const double z_bottom = ShapeLowestPoint(shape);
    const double z_top    = ShapeHighestPoint(shape);
    if (z_top - z_bottom < MIN_CONTACT_MM) return result;   // nothing to grip at all

    const double mu_limit = std::atan(cfg.friction_mu);

    for (double deg = 0.0; deg < 180.0; deg += ANGLE_STEP_DEG)
    {
        const double th = deg * M_PI / 180.0;
        const gp_Dir u(std::cos(th), std::sin(th), 0.0);
        const gp_Dir perp(-std::sin(th), std::cos(th), 0.0);

        // Project once per angle; the z sweep then just re-filters this.
        std::vector<std::pair<double, SurfPt>> proj;
        proj.reserve(pts.size());
        for (auto const& sp : pts)
            proj.emplace_back(sp.p.X() * u.X() + sp.p.Y() * u.Y(), sp);

        // Slide the jaw band up from the part's underside.  The jaw is allowed to
        // stand proud of the top — only the overlap touches, which is scored, not
        // forbidden — so the sweep runs until too little of the part is left to
        // grip.  The jaw may not drop below the part, where it would foul the jig.
        for (double z_lo = z_bottom; z_lo <= z_top - MIN_CONTACT_MM + 1e-9; z_lo += Z_STEP_MM)
        {
            const double z_hi = z_lo + cfg.jaw_height_mm;

            std::vector<std::pair<double, SurfPt>> band;
            band.reserve(proj.size() / 4);
            for (auto const& e : proj)
                if (e.second.p.Z() >= z_lo - 1e-9 && e.second.p.Z() <= z_hi + 1e-9)
                    band.push_back(e);
            if (band.size() < 3) continue;

            const Contact cp = findContact(band, u, perp, +1);
            const Contact cm = findContact(band, u, perp, -1);
            if (!cp.found || !cm.found) continue;

            const double width = cp.extent_a - cm.extent_a;
            if (width < cfg.min_opening_mm || width > cfg.max_opening_mm) continue;

            if (!feasible(cp, cm, mu_limit)) continue;
            const PPGContactClass k = classify(cp, cm, cfg);
            const double q = quality(cp, cm, cfg, mu_limit);

            // Grasp centre: midway between the contacts, mid-height of the jaws.
            const double a_mid = 0.5 * (cp.extent_a + cm.extent_a);
            // Recover a world point on the axis: the perpendicular coordinate is
            // taken through the centre of mass, which is also what keeps the
            // moment arm small.
            const double w_com = com.X() * perp.X() + com.Y() * perp.Y();
            const gp_Pnt centre(a_mid * u.X() + w_com * perp.X(),
                                a_mid * u.Y() + w_com * perp.Y(),
                                0.5 * (z_lo + z_hi));

            // Perpendicular distance from the CoM to the grasp axis — the lever
            // gravity acts on.  By construction of `centre` this is the axial
            // offset only.
            const double a_com = com.X() * u.X() + com.Y() * u.Y();
            const double moment = std::abs(a_com - a_mid);

            PPGCandidate c;
            c.angle_rad  = th;
            c.jaw_z_mm   = z_lo;
            c.width_mm   = width;
            c.centre     = centre;
            c.klass       = k;
            c.moment_arm  = moment;
            c.quality     = q;
            c.slope_rad   = std::max(cp.slope, cm.slope);
            c.contact_mm2 = std::min(cp.spread_z * cp.spread_w, cm.spread_z * cm.spread_w);
            result.candidates.push_back(c);
        }
    }

    // Rank by the trade-off score, breaking ties on the gravitational lever.
    std::sort(result.candidates.begin(), result.candidates.end(),
              [](const PPGCandidate& a, const PPGCandidate& b) {
                  if (std::abs(a.quality - b.quality) > 1e-9) return a.quality > b.quality;
                  return a.moment_arm < b.moment_arm;
              });

    return result;
}

// ---------------------------------------------------------------------------

TopoDS_Shape PPGGraspGenerator::jawSolid(const PPGCandidate&  cand,
                                         const GripperConfig& cfg,
                                         int                  side,
                                         double               sweep_to_z)
{
    const double th = cand.angle_rad;
    const gp_Dir u(std::cos(th), std::sin(th), 0.0);
    const gp_Dir perp(-std::sin(th), std::cos(th), 0.0);

    const double top = std::max(sweep_to_z, cand.jaw_z_mm + cfg.jaw_height_mm);

    // Build axis-aligned then rotate: the jaw spans its thickness along u, its
    // width along perp, and from the jaw underside up to `top`.
    const double a0 = side > 0
        ? 0.5 * cand.width_mm
        : -0.5 * cand.width_mm - cfg.jaw_thickness_mm;

    TopoDS_Shape box = BRepPrimAPI_MakeBox(
        gp_Pnt(a0, -0.5 * cfg.jaw_width_mm, cand.jaw_z_mm),
        gp_Pnt(a0 + cfg.jaw_thickness_mm, 0.5 * cfg.jaw_width_mm, top)).Shape();

    gp_Trsf rot;
    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), th);

    gp_Trsf move;
    // The box was built about the origin in the (u, perp) frame; place it on the
    // grasp axis.  Only the in-plane position matters, z is already absolute.
    const double a_c = cand.centre.X() * u.X() + cand.centre.Y() * u.Y();
    const double w_c = cand.centre.X() * perp.X() + cand.centre.Y() * perp.Y();
    move.SetTranslation(gp_Vec(a_c * u.X() + w_c * perp.X(),
                               a_c * u.Y() + w_c * perp.Y(),
                               0.0));

    gp_Trsf combined = move;
    combined.Multiply(rot);
    return BRepBuilderAPI_Transform(box, combined, true).Shape();
}

// ---------------------------------------------------------------------------

std::optional<PPGGrasp> PPGGraspGenerator::select(
    std::shared_ptr<Part>            part,
    const PartPPGCandidates&         candidates,
    const GripperConfig&             cfg,
    CollisionAdapter&                adapter,
    const std::vector<std::string>&  assembled_ids)
{
    if (!part || !part->getShape()) return std::nullopt;

    const TopoDS_Shape& shape = *part->getShape();
    const gp_Pnt centroid = ShapeCentroid(shape);
    const double clear_z  = ShapeHighestPoint(shape) + 30.0;   // descend from here

    const std::string probe = "__ppg_jaw__";

    for (auto const& c : candidates.candidates)
    {
        bool clear = true;

        // Sweep both jaws from clearance height down to the grasp: the jaws come
        // straight down, so the whole descent volume has to be free, not just the
        // final pose.
        for (int side : {+1, -1})
        {
            auto swept = std::make_shared<TopoDS_Shape>(
                jawSolid(c, cfg, side, clear_z));

            gp_Trsf identity;   // jawSolid is already in world mm
            auto lf = std::make_shared<TopoDS_Shape>(LocalFrameShapeM(*swept));
            const gp_Pnt sc = ShapeCentroid(*swept);
            gp_Trsf pose;
            pose.SetTranslation(gp_Vec(sc.X() * 0.001, sc.Y() * 0.001, sc.Z() * 0.001));

            adapter.add_or_update(probe, lf, pose);

            for (const auto& aid : assembled_ids)
            {
                if (!adapter.collision_free(probe, aid)) { clear = false; break; }
            }
            adapter.remove(probe);
            if (!clear) break;
        }

        if (!clear) continue;

        PPGGrasp g;
        g.position_ = gp_Vec(c.centre.X() - centroid.X(),
                             c.centre.Y() - centroid.Y(),
                             c.centre.Z() - centroid.Z());
        g.rotation_ = c.angle_rad;
        g.width_    = c.width_mm;
        g.jaw_z_    = c.jaw_z_mm - centroid.Z();
        g.valid_    = true;

        static const char* kNames[] = {"vertical face", "vertical line",
                                       "sloped face", "point contact"};
        RCLCPP_INFO(logger(),
                    "PPGGraspGenerator: grasp for %s  angle=%.1f deg  width=%.2f mm  "
                    "contact=%s (%.1f mm2, slope %.1f deg)  lever=%.2f mm  quality=%.2f",
                    part->getName().c_str(), c.angle_rad * 180.0 / M_PI, c.width_mm,
                    kNames[static_cast<int>(c.klass)], c.contact_mm2,
                    c.slope_rad * 180.0 / M_PI, c.moment_arm, c.quality);
        return g;
    }

    RCLCPP_WARN(logger(), "PPGGraspGenerator: no valid grasp for %s (%zu candidate(s) blocked)",
                part->getName().c_str(), candidates.candidates.size());
    return std::nullopt;
}
