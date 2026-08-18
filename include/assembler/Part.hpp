#ifndef PART_HPP
#define PART_HPP

#include "assembler/MeshAsset.hpp"
#include "assembler/MeshFunctions.hpp"
#include "assembler/PPGGraspGenerator.hpp"

class Part
{
public:

    enum PART_TYPE {
        NONE,
        INTERNAL,
        EXTERNAL,
        SCREW
    };
    
    Part(std::shared_ptr<TopoDS_Shape> shape, PART_TYPE type, size_t id, std::string name) : shape_(shape), type_(type), id_(id), name_(name) {}

    Part() = default;

    void translate(gp_Vec translation) {shape_ = std::make_shared<TopoDS_Shape>(TranslateShape(*shape_, translation));}

    bool collide(std::shared_ptr<Part> otherPart);

    double collisionVolume(std::shared_ptr<Part> otherPart);

    void setCentroidPosition(gp_Pnt position) {shape_ = std::make_shared<TopoDS_Shape>(ShapeSetCentroid(*shape_, position));}

    gp_Pnt generateBayPosition(std::vector<std::vector<bool>>& occupancy);

    // void generatePPGGraspPosition();

    void saveShape(std::string filename) {SaveShapeAsSTL(*shape_, filename);}

    void setVacuumGrasp(gp_Pnt grasp) { vacuum_grasp_position_ = grasp; }

    void setPPGGrasp(PPGGrasp grasp) { ppg_grasp_ = grasp; }

    gp_Pnt                          getVacuumGrasp()        { return vacuum_grasp_position_; }
    PPGGrasp                        getPPGGrasp()           { return ppg_grasp_; }

    void                            set_mesh_asset(std::shared_ptr<MeshAsset> m) { mesh_asset_ = m; }
    std::shared_ptr<MeshAsset>      get_mesh_asset() const  { return mesh_asset_; }

    gp_Pnt                          getCoM()                { return ShapeCenterOfMass(*shape_); }  //TODO maybe get rid of this, only in PPGGraspGenerator
    // gp_Pnt                          getCentroid()           { return ShapeCentroid(*shape_); }
    // Standard_Real                   getHighestPoint()       { return ShapeHighestPoint(*shape_); }

    PART_TYPE                       getType()               { return type_; }
    size_t                          getId()                 { return id_; }
    std::string                     getName()               { return name_; }
    bool                            isPushfit()             { return name_.find("pushfit") != std::string::npos; }
    std::shared_ptr<TopoDS_Shape>   getShape()              { return shape_; }

    int                             getBayIndex()           { return bay_index_; }
    int                             getBaySizeIndex()       { return bay_size_index_; }
    bool                            hasBayAssigned() const  { return has_bay_assigned_; }
    gp_Pnt                          getBayPosition() const  { return bay_position_; }

private:

    //TopoDS_Shape GenerateGripperPlate(gp_Dir normal, gp_Pnt center);

    std::shared_ptr<TopoDS_Shape> shape_;

    std::shared_ptr<MeshAsset> mesh_asset_;

    PART_TYPE type_;

    gp_Pnt vacuum_grasp_position_;  //Relative to CoM

    PPGGrasp ppg_grasp_;

    size_t id_;

    std::string name_;

    int bay_index_;

    int bay_size_index_;

    bool has_bay_assigned_ = false;
    gp_Pnt bay_position_;
};

#endif