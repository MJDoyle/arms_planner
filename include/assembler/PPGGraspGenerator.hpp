#ifndef PPG_GRASP_GENERATOR_HPP
#define PPG_GRASP_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

struct PPGGrasp
{
    gp_Vec position_;  //Relative to CoM

    Standard_Real rotation_;

    Standard_Real width_;
};

class Part;

class PPGGraspGenerator
{
    public: 

    static PPGGrasp generate(std::shared_ptr<Part> part);

    private:

    static TopoDS_Shape generateGripperPlate(gp_Dir normal, gp_Pnt center, TopoDS_Shape shape);
};

#endif