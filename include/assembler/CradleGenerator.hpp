#ifndef CRADLEGENERATOR_HPP
#define CRADLEGENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

struct Pt2 {
    double x, y;
};

class CradleGenerator
{
    public:

    CradleGenerator(std::string name, TopoDS_Shape shape, float scaling_distance = 0.2f)
        : shape_(shape), name_(name), scaling_distance_(scaling_distance) {}

    float createSimpleNegative(float bay_size, int bay_index, const std::string& output_dir);

    void createNegative();

    void createNegative2();

    TopTools_ListOfShape bottomEdgesFlatHorizontalFaces();

    gp_Vec outwardsNormalOfEdge(TopoDS_Edge edge, TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap);

    TopoDS_Shape createBoxWithPose(double width, double height, double length, gp_Pnt target_center, gp_Dir target_direction);

    private:

    TopoDS_Shape shape_;

    std::string name_;

    float scaling_distance_;
};


#endif