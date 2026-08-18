#ifndef MESH_FUNCTIONS_HPP
#define MESH_FUNCTIONS_HPP


#include <TDF_LabelSequence.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_Label.hxx>
#include <TDF_Tool.hxx>
#include <TDocStd_Document.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <Geom_Plane.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Handle.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <map>

#include <GeomAbs_SurfaceType.hxx>

#include <ShapeUpgrade_UnifySameDomain.hxx>

#include <TCollection_ExtendedString.hxx>
#include <TCollection_AsciiString.hxx>

#include <BRepExtrema_DistShapeShape.hxx>

#include <BRepBuilderAPI_MakeVertex.hxx>

#include <Extrema_ExtPS.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Precision.hxx>

#include <Geom_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopLoc_Location.hxx>

#include <BRepTopAdaptor_FClass2d.hxx>
#include <TopAbs_State.hxx>
#include <gp_Pnt2d.hxx>

#include <iostream>
#include <algorithm>
#include <cctype>
#include <string>

#include <memory>
#include <vector>

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>

#include <BRepAlgoAPI_Common.hxx>

#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Xw_Window.hxx> // On Windows; on Linux use Xw_Window

#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx> // Example shape

#include <StlAPI_Writer.hxx>

#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAlgoAPI_Cut.hxx>

#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Copy.hxx>

#include <TDF_ChildIterator.hxx>

#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>

#include <string>
#include <codecvt>
#include <locale>  // if you use codecvt

#include <BRepAlgoAPI_Section.hxx>

#include <Geom_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>

#include <BRepFilletAPI_MakeChamfer.hxx>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_Orientation.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <iostream>
#include <Standard_Type.hxx>

#include <BRepExtrema_DistShapeShape.hxx>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <STEPControl_Writer.hxx>

#include <gp_GTrsf.hxx>
#include <BRepBuilderAPI_GTransform.hxx>

#include <Standard_Real.hxx>

#include <BRepBuilderAPI_MakeFace.hxx>


#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>

#include <BOPAlgo_BOP.hxx>

#include <BRepProj_Projection.hxx>

#include <fstream>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

#include <BRepOffsetAPI_MakeOffsetShape.hxx>

#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>

extern const gp_Dir UPWARDS;

TopoDS_Shape ShapeIntersection(TopoDS_Shape shape_A, TopoDS_Shape shape_B);

Standard_Real ShapeVolume(TopoDS_Shape shape);

gp_Pnt ShapeCentroid(TopoDS_Shape shape);

TopoDS_Shape ShapeSetCentroid(TopoDS_Shape shape, gp_Pnt position);

gp_Pnt ShapeCenterOfMass(TopoDS_Shape shape);

Bnd_Box ShapeBoundingBox(TopoDS_Shape shape);

TopoDS_Shape ShapeBoundingBoxShape(TopoDS_Shape shape);

TopoDS_Shape ShapeHighBoundingBoxShape(TopoDS_Shape shape, Standard_Real height);

Standard_Real ShapeLowestPoint(TopoDS_Shape shape);

Standard_Real ShapeHighestPoint(TopoDS_Shape shape);

Standard_Real ShapeAxisSize(TopoDS_Shape shape, int axis);

TopoDS_Shape TranslateShape(TopoDS_Shape shape, gp_Vec vec);

TopoDS_Shape NonUniformScaleShape(TopoDS_Shape shape, gp_Pnt scaling);

TopoDS_Shape UniformScaleShape(TopoDS_Shape shape, Standard_Real scaling);

TopoDS_Shape SubtractShapeBFromA(TopoDS_Shape shape_A, TopoDS_Shape shape_B);

void SaveShapeAsSTL(TopoDS_Shape shape, std::string filename);

gp_Vec SumPoints(gp_Pnt point_A, gp_Pnt point_B);

gp_Vec SumPoints(gp_Pnt point_A, gp_Vec point_B);

gp_Vec SubtractPoints(gp_Pnt point_A, gp_Pnt point_B);

GeomAbs_CurveType getEdgeType(TopoDS_Edge edge);

gp_Pnt getEdgeStart(TopoDS_Edge edge);

gp_Pnt getEdgeEnd(TopoDS_Edge edge);

gp_Dir outwardFaceNormal(TopoDS_Face face);

double faceArea(const TopoDS_Face& face);

TopoDS_Shape makeCompound(const std::vector<TopoDS_Shape>& shapes);

#endif