#include "assembler/VacuumGraspGenerator.hpp"
#include "assembler/Logger.hpp"
#include "assembler/Part.hpp"

#include <cmath>
#include <limits>

// ---------------------------------------------------------------------------
// Vacuum grasp search using Coal BVH for all collision checks.
//
// Algorithm:
//   For each planar, upward-facing face with sufficient area:
//     Spiral outward from face centroid (radius × angle grid).
//     Place nozzle cylinder bottom 1 mm above face highest point —
//       the 1 mm gap prevents false collision with the face itself.
//     Accept the candidate if:
//       (a) nozzle is collision-free with the part body
//       (b) nozzle is collision-free with every assembled-part ID
//   Among all accepted candidates, return the one closest to the CoM.
//
// Nozzle geometry (matches VacuumGraspGenerator legacy dimensions):
//   radius 4.2 mm, height 20 mm, local-frame centroid at origin,
//   bottom at z = -10 mm, top at z = +10 mm.
//
// Returned grasp convention (unchanged from legacy):
//   Local-frame offset of the contact point (face surface) relative to the
//   part bounding-box centroid, in mm.  The nozzle centroid in world space is
//   at (centroid + grasp) + (0, 0, 10 mm).
// ---------------------------------------------------------------------------

std::optional<gp_Pnt> VacuumGraspGenerator::generate(
    std::shared_ptr<Part>              part,
    CollisionAdapter&                  adapter,
    const std::shared_ptr<MeshAsset>&  nozzle_mesh,
    const std::vector<std::string>&    assembled_ids,
    std::vector<GraspAttempt>*         debug_out)
{
    if (!nozzle_mesh)
        return std::nullopt;

    TopoDS_Shape shape = *(part->getShape());

    constexpr double NOZZLE_RADIUS  =  4.2;   // mm
    constexpr double NOZZLE_H_HALF  = 10.0;   // mm  (half of 20 mm)
    constexpr double NOZZLE_GAP_MM  =  1.0;   // mm above face — avoids face-surface false positive
    constexpr double FACE_MIN_AREA  = 50.0;   // mm²
    constexpr double ANGLE_TOL      = 0.05;   // rad — max deviation from vertical
    // Seal check: a cylinder of the same radius, 1 mm tall, placed half-inside
    // the contact surface.  Requires 98 % of its volume to intersect the part.
    constexpr double TIP_HEIGHT     =  1.0;   // mm
    constexpr double TIP_SEAL_FRAC  =  0.95;
    const double     tip_vol = M_PI * NOZZLE_RADIUS * NOZZLE_RADIUS * TIP_HEIGHT;

    const gp_Pnt shape_com      = ShapeCenterOfMass(shape);
    const gp_Pnt shape_centroid = ShapeCentroid(shape);

    const std::string part_id = part->getName() + "_" + std::to_string(part->getId());

    struct Candidate {
        double   com_dist;
        gp_Pnt   local_pos;  // grasp in local frame (contact point, mm)
    };
    std::vector<Candidate> candidates;

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face face = TopoDS::Face(exp.Current());

        // Only flat upward-facing faces with enough area.
        if (!BRep_Tool::Surface(face)->IsKind(STANDARD_TYPE(Geom_Plane))) continue;
        const gp_Dir normal = outwardFaceNormal(face);
        if (normal.Angle(UPWARDS) > ANGLE_TOL) continue;
        if (faceArea(face) < FACE_MIN_AREA)    continue;

        const gp_Pnt  face_cen     = ShapeCentroid(face);
        const double  face_top_z   = ShapeHighestPoint(face);
        const double  largest_axis = std::max(ShapeAxisSize(face, 0), ShapeAxisSize(face, 1));

        // Nozzle centroid Z for the Coal check: bottom 1 mm above face surface.
        const double nozzle_cen_z_mm = face_top_z + NOZZLE_GAP_MM + NOZZLE_H_HALF;

        bool found_on_face = false;

        for (double r = 0.0; r < largest_axis && !found_on_face; r += 0.5)
        {
            for (int th = 0; th < 360; th += 45)
            {
                const double nx = face_cen.X() + r * std::cos(th * M_PI / 180.0);
                const double ny = face_cen.Y() + r * std::sin(th * M_PI / 180.0);

                // Pose in metres (Tessellator stores local-frame mesh with centroid at origin).
                gp_Trsf pose;
                pose.SetTranslation(gp_Vec(nx * 0.001, ny * 0.001, nozzle_cen_z_mm * 0.001));
                adapter.add_or_update("__grasp_nozzle__", nozzle_mesh, pose);

                // (a) Nozzle body must not intersect the part.
                bool body_clear = adapter.collision_free("__grasp_nozzle__", part_id);

                // (b) Nozzle body must not intersect any assembled part.
                bool assembly_clear = true;
                if (body_clear)
                {
                    for (const auto& aid : assembled_ids)
                    {
                        if (!adapter.collision_free("__grasp_nozzle__", aid))
                        {
                            assembly_clear = false;
                            break;
                        }
                    }
                }

                adapter.remove("__grasp_nozzle__");

                if (!body_clear) {
                    if (debug_out)
                        debug_out->push_back({nx, ny, nozzle_cen_z_mm,
                                              GraspAttempt::Status::body_collision});
                    continue;
                }
                if (!assembly_clear) {
                    if (debug_out)
                        debug_out->push_back({nx, ny, nozzle_cen_z_mm,
                                              GraspAttempt::Status::assembly_collision});
                    continue;
                }

                // (c) Seal check (OCCT): a tip cylinder of the same radius (1 mm tall),
                //     positioned with its centroid 0.5 mm below the face surface, must
                //     overlap ≥ 95 % of its own volume with the part body.
                //     This runs only after both Coal checks pass — the cheap BVH filter
                //     ensures the OCCT Boolean is not reached for most positions.
                {
                    TopoDS_Shape tip = BRepPrimAPI_MakeCylinder(NOZZLE_RADIUS, TIP_HEIGHT).Shape();
                    tip = ShapeSetCentroid(tip, gp_Pnt(nx, ny, face_top_z - TIP_HEIGHT * 0.5));
                    TopoDS_Shape isect = ShapeIntersection(tip, shape);
                    if (isect.IsNull() || ShapeVolume(isect) / tip_vol < TIP_SEAL_FRAC) {
                        if (debug_out)
                            debug_out->push_back({nx, ny, nozzle_cen_z_mm,
                                                  GraspAttempt::Status::seal_failed});
                        continue;
                    }
                }

                if (debug_out)
                    debug_out->push_back({nx, ny, nozzle_cen_z_mm,
                                          GraspAttempt::Status::accepted});

                // Grasp = contact point in local frame (bottom of nozzle = face surface).
                const gp_Pnt local_pos(
                    nx          - shape_centroid.X(),
                    ny          - shape_centroid.Y(),
                    face_top_z  - shape_centroid.Z());

                const double com_dist = std::hypot(nx - shape_com.X(), ny - shape_com.Y());
                candidates.push_back({com_dist, local_pos});
                found_on_face = true;
            }
        }
    }

    if (candidates.empty())
    {
        RCLCPP_WARN(logger(), "VacuumGraspGenerator: no valid grasp for %s",
                    part->getName().c_str());
        return std::nullopt;
    }

    const auto best = std::min_element(
        candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b){ return a.com_dist < b.com_dist; });

    RCLCPP_INFO(logger(),
                "VacuumGraspGenerator: grasp for %s  pos=(%.2f, %.2f, %.2f)  CoM dist=%.2f",
                part->getName().c_str(),
                best->local_pos.X(), best->local_pos.Y(), best->local_pos.Z(),
                best->com_dist);

    return best->local_pos;
}
