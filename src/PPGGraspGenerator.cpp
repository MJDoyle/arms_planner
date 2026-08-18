#include "assembler/PPGGraspGenerator.hpp"

#include "assembler/Logger.hpp"

#include "assembler/Part.hpp"

#include "assembler/Config.hpp"

PPGGrasp PPGGraspGenerator::generate(std::shared_ptr<Part> part)
{
    struct Grasp
    {
        gp_Pnt center1;
        gp_Pnt center2;

        gp_Dir normal1;
        gp_Dir normal2;

        float com_xy_magnitude;

        float gripper_bottom_height;

        TopoDS_Compound padels_compound;
    };


    if (part->getType() == Part::PART_TYPE::SCREW)
        return PPGGrasp();

    RCLCPP_INFO(logger(), "Generating PPG grasp");

    TopoDS_Shape shape = *(part->getShape());

    //Iterate through the faces of the part and get their normals AND antinormals

    //Disregard normals that don't have close to zero value in z
    //Check all remaining normals against one another to find pairs that align, that are close to the CoM, and that don't cause collisions

    std::vector<gp_Pnt> centers;
    std::vector<gp_Dir> normals;

    for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        TopoDS_Face face = TopoDS::Face(faceExp.Current());
        BRepAdaptor_Surface surf(face);

        GeomAbs_SurfaceType surfType = surf.GetType();

        Standard_Real u1, u2, v1, v2;
        BRepTools::UVBounds(face, u1, u2, v1, v2);

        // Midpoint in parameter space
        Standard_Real uMid = (u1 + u2) / 2.0;
        Standard_Real vMid = (v1 + v2) / 2.0;

        // Compute derivatives
        gp_Pnt center;
        gp_Vec d1u, d1v;
        surf.D1(uMid, vMid, center, d1u, d1v);

        // Compute normal
        gp_Vec normal = d1u.Crossed(d1v);
        if (normal.Magnitude() < 1e-6) continue; // Skip degenerate

        normal.Normalize();

        // Create a small arrow representing the normal
        Standard_Real length = 10.0;  // Adjust length as needed
        gp_Pnt end = center.Translated(normal.Scaled(length));

        gp_Pnt otherEnd = center.Translated(-normal.Scaled(length));

        TopoDS_Edge normalEdge = BRepBuilderAPI_MakeEdge(center, end);
        Handle(AIS_Shape) aisNormal = new AIS_Shape(normalEdge);

        centers.push_back(center);
        normals.push_back(normal);

        centers.push_back(center);
        normals.push_back(-normal);
    }

    bool grasp_found = false;

    Grasp best_grasp;    

    for (int a = 0; a < normals.size(); a++)
    {
        for (int b = 0; b < normals.size(); b++)
        {
            Standard_Real normal_normal_angle = normals[a].Angle(normals[b]);

            //Check the the two normals are anti-aligned
            if (normal_normal_angle < 0.99 * 3.14159)
                continue;

            gp_Vec delta(centers[a], centers[b]);

            //Check the points are far enough away to grasp
            if (delta.Magnitude() < 1)
                continue;

            Standard_Real delta_normal_angle = gp_Vec(normals[b]).Angle(delta);

            //Check if the normal and the delta are misaligned
            if (delta_normal_angle > 0.01 * 3.14159)
                continue;

            //Check x y distance from center of delta to CoM
            gp_Pnt delta_center = centers[a].Translated(0.5 * delta);

            gp_Vec com_xy_distance(delta_center.X() - part->getCoM().X(), delta_center.Y() - part->getCoM().Y(), 0);

            float com_xy_mag = com_xy_distance.Magnitude();

            if (com_xy_mag > 5)
                continue;

            TopoDS_Shape gripper_plate_1 = generateGripperPlate(normals[a], centers[a], shape);
            TopoDS_Shape gripper_plate_2 = generateGripperPlate(normals[b], centers[b], shape);

            //Intersect the paddles and the part
            TopoDS_Shape intersection_1 = ShapeIntersection(gripper_plate_1, shape);

            TopoDS_Shape intersection_2 = ShapeIntersection(gripper_plate_2, shape);

            if ((!intersection_1.IsNull() && ShapeVolume(intersection_1) > 0.01) || (!intersection_2.IsNull() && ShapeVolume(intersection_2) > 0.01))
            {   
                //Collision between paddles and part, continue

                continue;
            }

    

            TopoDS_Compound compound;
            BRep_Builder builder;
            builder.MakeCompound(compound);

            builder.Add(compound, shape);
            builder.Add(compound, gripper_plate_1);
            builder.Add(compound, gripper_plate_2);

            // std::cout << std::endl << "GRASP: " << std::endl;
            // std::cout << "Face A center: " << centers[a].X() << ", " << centers[a].Y() << ", " << centers[a].Z() << std::endl;
            // std::cout << "Face A normal: " << normals[a].X() << ", " << normals[a].Y() << ", " << normals[a].Z() << std::endl;
            // std::cout << "Face B center: " << centers[b].X() << ", " << centers[b].Y() << ", " << centers[b].Z() << std::endl;
            // std::cout << "Face B normal: " << normals[b].X() << ", " << normals[b].Y() << ", " << normals[b].Z() << std::endl << std::endl;;

            if (!grasp_found)
            {
                best_grasp = Grasp();

                best_grasp.center1 = centers[a];
                best_grasp.center2 = centers[b];

                best_grasp.normal1 = normals[a];
                best_grasp.normal2 = normals[b];

                best_grasp.gripper_bottom_height = ShapeLowestPoint(gripper_plate_1);

                best_grasp.padels_compound = compound;

                best_grasp.com_xy_magnitude = com_xy_mag;
            }

            //Compare grasps
            else if (com_xy_mag < best_grasp.com_xy_magnitude)
            {
                best_grasp = Grasp();

                best_grasp.center1 = centers[a];
                best_grasp.center2 = centers[b];

                best_grasp.normal1 = normals[a];
                best_grasp.normal2 = normals[b];

                best_grasp.gripper_bottom_height = ShapeLowestPoint(gripper_plate_1);

                best_grasp.padels_compound = compound;

                best_grasp.com_xy_magnitude = com_xy_mag;
            }



            grasp_found = true;


        }
    }

    if (grasp_found)
    {
        gp_Vec grasp_center = 0.5 * SumPoints(best_grasp.center2, best_grasp.center1);

        // std::cout << "grasp_center1: " << best_grasp.center1.X() << " " << best_grasp.center1.Y() << " " << best_grasp.center1.Z() << std::endl;
 
        // std::cout << "grasp_center2: " << best_grasp.center2.X() << " " << best_grasp.center2.Y() << " " << best_grasp.center2.Z() << std::endl;

        // std::cout << "grasp_center: " << grasp_center.X() << " " << grasp_center.Y() << " " << grasp_center.Z() << std::endl;

        gp_Vec default_angle_vector(0, 1, 0);

        Standard_Real grasp_angle = best_grasp.normal1.Angle(default_angle_vector);
        
        Standard_Real grasp_width = SubtractPoints(best_grasp.center2, best_grasp.center1).Magnitude();

        //Standard_Real plate_lowest_point = (ShapeLowestPoint(gripper_plate));

        //Calculate the height offset required due to the size of the part being picked up
        
        float height_offset = 0.432662 - (0.05623799 / -0.08505999) * (1 - exp(0.08505999 * grasp_width));

        Standard_Real grasp_height = best_grasp.gripper_bottom_height + height_offset;

        Standard_Real shape_lowest_point = ShapeLowestPoint(shape);

        Standard_Real lowest_point_delta = shape_lowest_point - grasp_height;


        if (lowest_point_delta > -0.5)
        {
            grasp_height += lowest_point_delta + 0.5;
        }

        PPGGrasp found_grasp = PPGGrasp();

        found_grasp.position_ = SubtractPoints(gp_Pnt(grasp_center.X(), grasp_center.Y(), grasp_height), part->getCoM());

        found_grasp.rotation_ = grasp_angle;

        found_grasp.width_ = grasp_width;

        // std::cout << "Grasp pos: " << ppg_grasp_position_.X() << " " << ppg_grasp_position_.Y() << " " << ppg_grasp_position_.Z() << std::endl;
        // std::cout << "Grasp angle: " << grasp_angle << std::endl;
        // std::cout << "Grasp width: " << grasp_width << std::endl;
 
 

        BRepMesh_IncrementalMesh mesher(best_grasp.padels_compound, 0.1);

        StlAPI_Writer writer;

        std::stringstream ss;

        ss << WORKING_DIR << part->getName() << "_ppg_grasp.stl";

        writer.Write(best_grasp.padels_compound, ss.str().c_str());

        return found_grasp;
    }

    return PPGGrasp();

    //TODO need to check collisions and 'size' of grasp
}

TopoDS_Shape PPGGraspGenerator::generateGripperPlate(gp_Dir normal, gp_Pnt center, TopoDS_Shape shape)
{
    TopoDS_Shape gripper_plate = BRepPrimAPI_MakeBox(10, 4, 12).Shape();
    gp_Dir source_normal(0, 1, 0);
    gp_Dir target_normal = normal;
    gp_Pnt source_point(5, 0, 5);
    gp_Pnt target_point = center.Translated(1.1 * gp_Vec(normal));
    gp_Vec rotation_vector;
    Standard_Real rotation_angle;
    gp_Trsf rotation;
    gp_Trsf translation;

    if ((gp_Vec(source_normal) ^ gp_Vec(target_normal)).Z() > 0)
        rotation_vector = gp_Vec(0, 0, 1);

    else
        rotation_vector = gp_Vec(0, 0, -1);

    rotation_angle = source_normal.Angle(target_normal); 

    gp_Ax1 rotation_axis(source_point, gp_Dir(rotation_vector));  // rotate around axis passing through source point
    rotation.SetRotation(rotation_axis, rotation_angle);

    BRepBuilderAPI_Transform rotTransformer(rotation);
    rotTransformer.Perform(gripper_plate);
    gripper_plate = rotTransformer.Shape();

    gp_Pnt rotated_source_point = source_point.Transformed(rotation);  // new location of P1 after rotation TODO requierd?

    gp_Vec translation_vector(rotated_source_point, target_point);

    translation.SetTranslation(translation_vector);

    BRepBuilderAPI_Transform transTransformer(translation);
    transTransformer.Perform(gripper_plate);
    gripper_plate = transTransformer.Shape();

    //Check bottom of gripper plate and move it up slightly above part
    Standard_Real shape_lowest_point = ShapeLowestPoint(shape);

    Standard_Real plate_lowest_point = ShapeLowestPoint(gripper_plate);

    Standard_Real lowest_point_delta = shape_lowest_point - plate_lowest_point;

    if (lowest_point_delta > -0.5)
    {
        gp_Vec raise_vector(0, 0, lowest_point_delta + 0.5);

        gp_Trsf raise_translation;

        raise_translation.SetTranslation(raise_vector);

        BRepBuilderAPI_Transform raise_transformer(raise_translation);
        raise_transformer.Perform(gripper_plate);
        gripper_plate = raise_transformer.Shape();
    }

    return gripper_plate;
}