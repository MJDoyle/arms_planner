#include "assembler/MeshFunctions.hpp"

const gp_Dir UPWARDS = gp_Dir(0, 0, 1);

TopoDS_Shape ShapeIntersection(TopoDS_Shape shape_A, TopoDS_Shape shape_B)
{
    return BRepAlgoAPI_Common(shape_A, shape_B);
}

Standard_Real ShapeVolume(TopoDS_Shape shape)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
}

gp_Pnt ShapeCentroid(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return gp_Pnt((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2);
}

TopoDS_Shape ShapeSetCentroid(TopoDS_Shape shape, gp_Pnt position)
{
    gp_Pnt current_centroid = ShapeCentroid(shape);

    gp_Vec move(current_centroid, position);

    TopoDS_Shape moved_shape = TranslateShape(shape, move);

    return moved_shape;
}

gp_Pnt ShapeCenterOfMass(TopoDS_Shape shape)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);

    return props.CentreOfMass();
}

Bnd_Box ShapeBoundingBox(TopoDS_Shape shape)
{
    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);

    return bbox;
}

TopoDS_Shape ShapeBoundingBoxShape(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    TopoDS_Shape bbox_shape = BRepPrimAPI_MakeBox(xmax - xmin, ymax - ymin, zmax - zmin).Shape();

    gp_Pnt shape_centroid = ShapeCentroid(shape);

    gp_Pnt bbox_centroid = ShapeCentroid(bbox_shape);

    gp_Vec move(bbox_centroid, shape_centroid);

    bbox_shape = TranslateShape(bbox_shape, move);

    return bbox_shape;
}

TopoDS_Shape ShapeHighBoundingBoxShape(TopoDS_Shape shape, Standard_Real height)
{
    std::cout << "check1" << std::endl;

    Bnd_Box bbox = ShapeBoundingBox(shape);

    std::cout << "check2" << std::endl;

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    std::cout << "check3" << std::endl;

    TopoDS_Shape bbox_shape = BRepPrimAPI_MakeBox(xmax - xmin, ymax - ymin, height).Shape();

    std::cout << "check4" << std::endl;

    gp_Pnt shape_centroid = ShapeCentroid(shape);

    std::cout << "check5" << std::endl;

    gp_Pnt bbox_centroid = ShapeCentroid(bbox_shape);

    std::cout << "check6" << std::endl;

    Standard_Real bbox_zmin = ShapeLowestPoint(bbox_shape);

    std::cout << "check7" << std::endl;

    gp_Vec move(shape_centroid.X() - bbox_centroid.X(), shape_centroid.Y() - bbox_centroid.Y(), zmin - bbox_zmin);
    
    bbox_shape = TranslateShape(bbox_shape, move);

    std::cout << "check8" << std::endl;

    return bbox_shape;
}

Standard_Real ShapeLowestPoint(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return zmin;
}

Standard_Real ShapeHighestPoint(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return zmax;
}

Standard_Real ShapeAxisSize(TopoDS_Shape shape, int axis)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    if (axis == 0)
        return xmax - xmin;

    if (axis == 1)
        return ymax - ymin;

    if (axis == 2)
        return zmax - zmin;

    std::cerr << "Axis not available" << std::endl;

    return 0;
}

TopoDS_Shape TranslateShape(TopoDS_Shape shape, gp_Vec vec)
{
    gp_Trsf move_trsf;

    move_trsf.SetTranslation(vec);

    return shape.Moved(move_trsf);
}

TopoDS_Shape NonUniformScaleShape(TopoDS_Shape shape, gp_Pnt scaling)
{
    std::cout << "Scaling shape" << std::endl;

    gp_GTrsf gtrsf;
    gtrsf.SetValue(1, 1, scaling.X());
    gtrsf.SetValue(2, 2, scaling.Y());
    gtrsf.SetValue(3, 3, scaling.Z());

    return BRepBuilderAPI_GTransform(shape, gtrsf, true).Shape();
}

TopoDS_Shape UniformScaleShape(TopoDS_Shape shape, Standard_Real scaling)
{
    gp_Pnt scaleCenter = ShapeCentroid(shape);

    gp_Trsf trsf;
    trsf.SetScale(scaleCenter, scaling);

    return BRepBuilderAPI_Transform(shape, trsf, true).Shape();
}

TopoDS_Shape SubtractShapeBFromA(TopoDS_Shape shape_A, TopoDS_Shape shape_B)
{
    // Perform the subtraction: A - B
    BRepAlgoAPI_Cut cut(shape_A, shape_B);

    cut.Build();

    if (!cut.IsDone()) {
        std::cerr << "Cut operation failed!" << std::endl;
    }

    return cut.Shape();
}

void SaveShapeAsSTL(TopoDS_Shape shape, std::string filename)
{
    BRepMesh_IncrementalMesh(shape, 0.5, false, 0.1, true);  // Mesh with a 0.1 tolerance

    // Export the result to STL
    StlAPI_Writer writer;
    writer.Write(shape, filename.c_str());
}

gp_Vec SumPoints(gp_Pnt point_A, gp_Pnt point_B)
{
    return gp_Vec(point_A.X() + point_B.X(), point_A.Y() + point_B.Y(), point_A.Z() + point_B.Z());
}

gp_Vec SumPoints(gp_Pnt point_A, gp_Vec point_B)
{
    return gp_Vec(point_A.X() + point_B.X(), point_A.Y() + point_B.Y(), point_A.Z() + point_B.Z());
}

gp_Vec SubtractPoints(gp_Pnt point_A, gp_Pnt point_B)
{
    return gp_Vec(point_A.X() - point_B.X(), point_A.Y() - point_B.Y(), point_A.Z() - point_B.Z());
}

GeomAbs_CurveType getEdgeType(TopoDS_Edge edge)
{
    Standard_Real first, last;

    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    if (curve.IsNull())
    {
        std::cout << "NULL CURVE" << std::endl;

        return GeomAbs_CurveType::GeomAbs_Line;
    }

    GeomAdaptor_Curve adaptor(curve, first, last);
    GeomAbs_CurveType type = adaptor.GetType();

    return type;    
}

gp_Pnt getEdgeStart(TopoDS_Edge edge)
{
    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    if (curve.IsNull())
    {
        std::cout << "NULL CURVE" << std::endl;

        return gp_Pnt(0, 0, 0);
    }

    GeomAdaptor_Curve adaptor(curve, first, last);

    return curve->Value(first);
}

gp_Pnt getEdgeEnd(TopoDS_Edge edge)
{
    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    if (curve.IsNull())
    {
        std::cout << "NULL CURVE" << std::endl;

        return gp_Pnt(0, 0, 0);
    }

    GeomAdaptor_Curve adaptor(curve, first, last);

    return curve->Value(last);
}

gp_Dir outwardFaceNormal(TopoDS_Face face)
{
    gp_Dir normal;
    Standard_Real umin, umax, vmin, vmax;

    // Pick a parameter on the face:
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    Standard_Real u = (umin + umax) / 2;
    Standard_Real v = (vmin + vmax) / 2;

    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

    // Get geometric normal (could be inward)
    GeomLProp_SLProps props(surf, u, v, 1, 1e-6);
    if(props.IsNormalDefined()) {
        normal = props.Normal();
    }

    // Now apply the face orientation:
    if(face.Orientation() == TopAbs_REVERSED)
        normal.Reverse();

    return normal;
}

double faceArea(const TopoDS_Face& face)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.Mass();
}

TopoDS_Shape makeCompound(const std::vector<TopoDS_Shape>& shapes)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    for (const auto& shape : shapes)
    {
        builder.Add(compound, shape);
    }

    return compound;
}

// void ProjectedContourFromShape(TopoDS_Shape shape, gp_Pnt origin, gp_Dir normal)
// {
//     gp_Pln plane(origin, normal);

//     TopoDS_Face planeFace = BRepBuilderAPI_MakeFace(plane);
//     BRepAlgoAPI_Section sectionAlgo(shape, planeFace, false);
//     sectionAlgo.ComputePCurveOn1(Standard_True);
//     sectionAlgo.Build();

//     TopoDS_Shape sectionShape = sectionAlgo.Shape();  // Contains edges on the projection plane

//     std::cout << "check1" << std::endl;

//     // 2. Collect edges
//     BRepBuilderAPI_MakeWire wireBuilder;
//     for (TopExp_Explorer ex(sectionShape, TopAbs_EDGE); ex.More(); ex.Next()) {
//         TopoDS_Edge edge = TopoDS::Edge(ex.Current());
//         wireBuilder.Add(edge);
//     }

//     std::cout << "check2" << std::endl;

//     TopoDS_Wire wire = wireBuilder.Wire();

//     std::cout << "check3" << std::endl;

//     // 3. Create face
//     TopoDS_Face face = BRepBuilderAPI_MakeFace(wire);

//     std::cout << "check4" << std::endl;

//     // 4. Extrude
//     gp_Vec extrusionVec(0, 0, 10);  // change vector as needed
//     TopoDS_Shape prism = BRepPrimAPI_MakePrism(face, extrusionVec);

//     std::cout << "check5" << std::endl;

//     SaveShapeAsSTL(prism, "section_test.stl");

//     std::cout << "check6" << std::endl;
// }

