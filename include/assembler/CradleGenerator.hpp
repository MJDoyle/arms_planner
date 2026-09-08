#ifndef CRADLEGENERATOR_HPP
#define CRADLEGENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

#include <vector>

struct Pt2 {
    double x, y;
};

class CradleGenerator
{
    public:

    CradleGenerator(std::string name, TopoDS_Shape shape, float scaling_distance = 0.2f)
        : shape_(shape), name_(name), scaling_distance_(scaling_distance) {}

    // Cut channels through the cradle so parallel-plate jaws can reach the part.
    // A cradle is a snug negative of the part, so without these the jaws have
    // nowhere to go — the pocket wall sits exactly where a finger needs to be.
    // Each solid is the volume one jaw sweeps on its way down, already positioned
    // in the same frame as the part.
    void setFingerSlots(const std::vector<TopoDS_Shape>& slots) { finger_slots_ = slots; }

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

    std::vector<TopoDS_Shape> finger_slots_;
};


#endif