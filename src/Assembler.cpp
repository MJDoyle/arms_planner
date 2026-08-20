#include "assembler/Assembler.hpp"
#include "assembler/OcctCollisionAdapter.hpp"
#include "assembler/Tessellator.hpp"

#include "assembler/Assembly.hpp"

#include "assembler/CradleGenerator.hpp"
#include "assembler/AdvancedCradleGenerator.hpp"
#include "assembler/TestCradleGenerator.hpp"
#include "assembler/SparseCradleGenerator.hpp"
#include "assembler/CradleGenerator.hpp"

#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"
#include "assembler/MeshFunctions.hpp"

#include "yaml-cpp/yaml.h"

#include "assembler/Config.hpp"

#include <iostream>
#include <fstream>
#include <queue>
#include <set>
#include <stack>
#include <map>

#include <cstdlib>
#include <filesystem>
#include <limits>

#include "assembler/Part.hpp"

#include "assembler/Logger.hpp"

#include "assembler/VacuumGraspGenerator.hpp"

#include "assembler/PPGGraspGenerator.hpp"

#include "assembler/GCodeGenerator.hpp"

#include <stdexcept>

Assembler::Assembler()
{  
    initialisePartBays();
}

void Assembler::reset()
{
    target_assembly_.reset();
    initial_assembly_.reset();
    assembly_path_.clear();
    base_part_.reset();
    node_id_map_.clear();
    next_node_ID_ = 0;
    slicer_gcode_.clear();
    bay_occupancy_.clear();
    name_.clear();
    generate_grasps_ = true;
    generate_jigs_ = true;
    collision_volume_threshold_ = 0.0;
    tool_config_ = {};
    scene_.clear();
    collision_adapter_.reset();
    nozzle_mesh_.reset();
    nozzle_shape_.reset();

    initialisePartBays();
}

/*  Initialise the 2D occupancy values for the grid of parts bays - outer vector corresponds to bays of different sizes, inner vector to individual bays
*/
void Assembler::initialisePartBays()
{
    for (std::vector<gp_Pnt> bays : PARTS_BAY_POSITIONS)
    {
        bay_occupancy_.push_back(std::vector<bool>());

        for (gp_Pnt bay_position : bays)
        {
            bay_occupancy_.back().push_back(false);
        }
    }
}

void Assembler::saveAssemblyPath()
{
    YAML::Node root;

    // Assembly order only — no positions (those live in assembly_plan.yaml).
    // Each step records which part IDs are assembled vs unassembled at that point.
    YAML::Node order_node = YAML::Node(YAML::NodeType::Sequence);
    for (auto node : assembly_path_) {
        YAML::Node step_yaml;
        YAML::Node assembled_ids = YAML::Node(YAML::NodeType::Sequence);
        for (auto const& [part, _unused] : node->assembly_->getAssembledPartTransforms())
            assembled_ids.push_back(static_cast<int>(part->getId()));
        step_yaml["assembled"] = assembled_ids;
        YAML::Node unassembled_ids = YAML::Node(YAML::NodeType::Sequence);
        for (auto const& [part, _unused] : node->assembly_->getUnassembledPartTransforms())
            unassembled_ids.push_back(static_cast<int>(part->getId()));
        step_yaml["unassembled"] = unassembled_ids;
        order_node.push_back(step_yaml);
    }
    root["assembly_order"] = order_node;

    if (!assembly_path_.empty()) {
        YAML::Node part_names_node = YAML::Node(YAML::NodeType::Map);
        auto& first = assembly_path_.front();
        for (auto const& [part, _unused] : first->assembly_->getUnassembledPartTransforms())
            part_names_node[std::to_string(part->getId())] = part->getName();
        for (auto const& [part, _unused] : first->assembly_->getAssembledPartTransforms())
            part_names_node[std::to_string(part->getId())] = part->getName();
        root["part_names"] = part_names_node;
    }

    std::stringstream ss;

    ss << run_output_dir_ << name_ << "_assembly_cache.yaml";

    std::ofstream fout(ss.str());

    fout << root;
    fout.close();
}

bool Assembler::loadAssemblyPath()
{
    std::stringstream ss;

    ss << run_output_dir_ << name_ << "_assembly_cache.yaml";

    std::ifstream fin(ss.str());
    if (!fin) {
        return false;
    }
    YAML::Node root = YAML::Load(fin);
    assembly_path_.clear();
    for (auto node_yaml : root["path"]) {
        auto assembly = std::make_shared<Assembly>();
        for (auto id_pos : node_yaml["assembled"]) {
            size_t id = std::stoul(id_pos.first.as<std::string>());
            auto pos_list = id_pos.second;
            gp_Pnt pos(pos_list[0].as<double>(), pos_list[1].as<double>(), pos_list[2].as<double>());
            auto part = initial_assembly_->getPartById(id);
            assembly->setAssembledPart(part, pos);
        }
        for (auto id_pos : node_yaml["unassembled"]) {
            size_t id = std::stoul(id_pos.first.as<std::string>());
            auto pos_list = id_pos.second;
            gp_Pnt pos(pos_list[0].as<double>(), pos_list[1].as<double>(), pos_list[2].as<double>());
            auto part = initial_assembly_->getPartById(id);
            assembly->setUnassembledPart(part, pos);
        }
        auto assembly_node = std::make_shared<AssemblyNode>();
        assembly_node->assembly_ = assembly;
        assembly_path_.push_back(assembly_node);
    }
    return true;
}

/*  The main assembler function. Generates grasps and cradles for each part as appropriate, determines the correct order of assembly, and produces a .yaml command file to be sent to ARMS
*/
void Assembler::generateAssemblySequence()
{
    RCLCPP_INFO(logger(), "Starting assembler analysis");

    if (target_assembly_ == nullptr)
        throw std::runtime_error("Assembler: no target assembly set before generateAssemblySequence()");

    generateInitialAssembly();

    // Jig/grasp-only mode: skip path planning and all path-dependent steps
    if (!generate_path_)
    {
        RCLCPP_INFO(logger(), "Path planning disabled — running jig/grasp generation only");
        generateInitialPartPositions();

        if (generate_grasps_)
            generateGrasps();

        if (generate_jigs_)
            generateNegatives();

        return;
    }

    //For now, at least one part must be internal (printed)
    if (target_assembly_->getNumInternalParts() == 0)
        throw std::runtime_error("Assembler: target assembly has no internal (printed) parts");

    //if (!loadAssemblyPath()) {

        RCLCPP_INFO(logger(), "No cached path found, creating new one");

        //assembly_path_ = breadthFirstZAssembly();   //All parts in the path are in the target_assembly position
        assembly_path_ = depthFirstZAssembly();   //All parts in the path are in the target_assembly position
        saveAssemblyPath();

        // Propagate the edge grasps found during DFS onto each part so that
        // downstream steps (generateCommandFile) can use part->getVacuumGrasp().
        if (generate_grasps_)
        {
            for (const auto& node : assembly_path_)
            {
                if (node->edge_part_ && node->edge_part_->getType() == Part::EXTERNAL)
                    node->edge_part_->setVacuumGrasp(node->edge_grasp_);
            }
        }
    //}

    //else
    //{
    //    RCLCPP_INFO(logger(), "Loading cached path");
    //}

    if (assembly_path_.size() < 2)
    {
        RCLCPP_WARN(logger(), "Assembly path has fewer than 2 nodes — skipping path-dependent steps");
        return;
    }

    //First node on assembly path has no assembled parts
    //Second node has one assembled part - this should be internal and is the base part

    if (assembly_path_[1]->assembly_->getAssembledPartTransforms().size() != 1)
    {
        RCLCPP_WARN(logger(), "Second assembly node doesn't have exactly one assembled part — skipping path-dependent steps");
        return;
    }

    if (assembly_path_[1]->assembly_->getAssembledPartTransforms().begin()->first->getType() != Part::PART_TYPE::INTERNAL)
    {
        RCLCPP_WARN(logger(), "Second assembly node part isn't internal — skipping path-dependent steps");
        return;
    }

    base_part_ = assembly_path_[1]->assembly_->getAssembledPartTransforms().begin()->first;  //Set base part

    //TODO
    //Now that you have the base part, check all other parts and compare their lowest z point to the lowest z point of the base part
    //For any parts that are below the base part, you need to move all parts together up by the difference plus some offset
    //Then, create a raft to print the base part on. This can either be exactly the bottom face of the base part or, (easier) just a bounding box
    //You need some small offest between the top of the raft and the bottom of the base part. You can maybe also use ribs like in the chatgpt discussion

    //REMEMBER that you need to get the part transforms from the assembly map - you can't just use shapegetlowestpoint


    generateInitialPartPositions(); //Initial part positions are now set here   Sets inital_assembly positions

    if (generate_grasps_)
        generateGrasps();

    RCLCPP_INFO(logger(), "generate_jigs_=%d  generate_grasps_=%d", (int)generate_jigs_, (int)generate_grasps_);
    if (generate_jigs_)
        generateNegatives();

    generateSlicerGcode();  //Uses initial_assembly positions

    alignAssemblyPathToInitialAssembly();


    debugState();


    std::vector<size_t> part_addition_order = generatePartAdditionOrder();

    generateGCodeFile(part_addition_order);

    generateCommandFile(part_addition_order);
}

void Assembler::debugState()
{
    RCLCPP_INFO(logger(), "Base part ID: %ld", base_part_->getId());

    RCLCPP_INFO(logger(), "INITIAL STATE");

    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        RCLCPP_INFO(logger(), "Part Id: %ld Position: %f, %f, %f", part->getId(), initial_transform.X(), initial_transform.Y(), initial_transform.Z());
    }

    RCLCPP_INFO(logger(), "TARGET STATE");

    for (auto const& [part, target_transform] : target_assembly_->getAssembledPartTransforms())  //All parts in target assembly are assembled
    {
        RCLCPP_INFO(logger(), "Part Id: %ld Position: %f, %f, %f", part->getId(), target_transform.X(), target_transform.Y(), target_transform.Z());
    }

    RCLCPP_INFO(logger(), "ASSEMBLY PATH");

    for (std::shared_ptr<AssemblyNode> node : assembly_path_)
    {
        RCLCPP_INFO(logger(), "NODE");

        for (auto const& [part, transform] : node->assembly_->getUnassembledPartTransforms())  
        {
            RCLCPP_INFO(logger(), "Unassembled part Id: %ld Position: %f, %f, %f", part->getId(), transform.X(), transform.Y(), transform.Z());
        }

        for (auto const& [part, transform] : node->assembly_->getAssembledPartTransforms()) 
        {
            RCLCPP_INFO(logger(), "Assembled part Id: %ld Position: %f, %f, %f", part->getId(), transform.X(), transform.Y(), transform.Z());
        }
    }

}

/*
Generates a direct GCode file that can be uploaded to ARMS and 'printed' directly
*/
void Assembler::generateGCodeFile(std::vector<size_t> part_addition_order)
{
    GCodeGenerator::generate(initial_assembly_, target_assembly_, base_part_, part_addition_order, slicer_gcode_, run_output_dir_);
}

/*  Uses the grasps, part positions and path to generate the command file to be sent to ARMS
*/
void Assembler::generateCommandFile(std::vector<size_t> part_addition_order)
{
    //In these commands for now let's give the part-height as the height of the pnp location relative to 0,
    //and the z location as the height of the pnp placement location relative to 0
    //You then might need to offset by the vacuum toolhead offset at some point

    YAML::Node root;

    YAML::Node commands = YAML::Node(YAML::NodeType::Sequence);


    //Designate internal parts
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        if (part->getType() != Part::INTERNAL)
            continue;   

        gp_Vec pick_position = SumPoints(initial_transform, part->getVacuumGrasp());

        gp_Vec place_position = SumPoints(target_assembly_->getAssembledPartTransforms()[part], part->getVacuumGrasp());

        PPGGrasp ppg_grasp = part->getPPGGrasp();

        gp_Vec grasp_position = SumPoints(initial_transform, ppg_grasp.position_);

        RCLCPP_INFO(logger(), "Pick pos: %f, %f, %f", pick_position.X(), pick_position.Y(), pick_position.Z());

        RCLCPP_INFO(logger(), "Place pos: %f, %f, %f", place_position.X(), place_position.Y(), place_position.Z());

        const gp_Pnt assembled_pos_int = target_assembly_->getAssembledPartTransforms()[part];

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_INTERNAL_PART";
        designate_part_command["command-properties"]["part-id"] = part->getId();
        designate_part_command["command-properties"]["part-name"] = part->getName();
        designate_part_command["command-properties"]["part-centroid-unassembled"]["x"] = initial_transform.X();
        designate_part_command["command-properties"]["part-centroid-unassembled"]["y"] = initial_transform.Y();
        designate_part_command["command-properties"]["part-centroid-unassembled"]["z"] = initial_transform.Z();
        designate_part_command["command-properties"]["part-centroid-assembled"]["x"] = assembled_pos_int.X();
        designate_part_command["command-properties"]["part-centroid-assembled"]["y"] = assembled_pos_int.Y();
        designate_part_command["command-properties"]["part-centroid-assembled"]["z"] = assembled_pos_int.Z();
        designate_part_command["command-properties"]["part-pick-height"] = pick_position.Z();
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z();
        designate_part_command["command-properties"]["part-pick-pos-x"] = pick_position.X();
        designate_part_command["command-properties"]["part-pick-pos-y"] = pick_position.Y();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();
        designate_part_command["command-properties"]["part-grasp-height"] = grasp_position.Z();
        designate_part_command["command-properties"]["part-grasp-pos-x"] = grasp_position.X();
        designate_part_command["command-properties"]["part-grasp-pos-y"] = grasp_position.Y();
        designate_part_command["command-properties"]["part-grasp-angle"] = ppg_grasp.rotation_;
        designate_part_command["command-properties"]["part-grasp-width"] = ppg_grasp.width_;

        commands.push_back(designate_part_command);
    }

    //Designate external parts
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {        
        if (part->getType() != Part::EXTERNAL)
            continue;

        gp_Vec pick_position = SumPoints(initial_transform, part->getVacuumGrasp());

        gp_Vec place_position = SumPoints(target_assembly_->getAssembledPartTransforms()[part], part->getVacuumGrasp());

        const gp_Pnt assembled_pos_ext = target_assembly_->getAssembledPartTransforms()[part];

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_EXTERNAL_PART";
        designate_part_command["command-properties"]["part-id"] = part->getId();
        designate_part_command["command-properties"]["part-name"] = part->getName();
        designate_part_command["command-properties"]["part-centroid-unassembled"]["x"] = initial_transform.X();
        designate_part_command["command-properties"]["part-centroid-unassembled"]["y"] = initial_transform.Y();
        designate_part_command["command-properties"]["part-centroid-unassembled"]["z"] = initial_transform.Z();
        designate_part_command["command-properties"]["part-centroid-assembled"]["x"] = assembled_pos_ext.X();
        designate_part_command["command-properties"]["part-centroid-assembled"]["y"] = assembled_pos_ext.Y();
        designate_part_command["command-properties"]["part-centroid-assembled"]["z"] = assembled_pos_ext.Z();
        designate_part_command["command-properties"]["part-pick-height"] = pick_position.Z();
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z();
        designate_part_command["command-properties"]["part-pick-pos-x"] = pick_position.X();
        designate_part_command["command-properties"]["part-pick-pos-y"] = pick_position.Y();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();
        designate_part_command["command-properties"]["grasp-pos-x"] = part->getVacuumGrasp().X();
        designate_part_command["command-properties"]["grasp-pos-y"] = part->getVacuumGrasp().Y();
        designate_part_command["command-properties"]["grasp-pos-z"] = part->getVacuumGrasp().Z();

        commands.push_back(designate_part_command);
    }

    //Designate screws
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        if (part->getType() != Part::SCREW)
            continue;

        gp_Pnt place_position(target_assembly_->getAssembledPartTransforms()[part].X(), 
                                target_assembly_->getAssembledPartTransforms()[part].Y(), 
                                target_assembly_->getAssembledPartTransforms()[part].Z() + 5); //TODO need to work out what to do with screw z offset

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_SCREW";
        designate_part_command["command-properties"]["part-id"] = part->getId();
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();

        commands.push_back(designate_part_command);
    }

    //Print all internal parts together
    if (initial_assembly_->getNumInternalParts() != 0)
    {
        YAML::Node direct_print_command;
        direct_print_command["command-type"] = "DIRECT_PRINT";

        // Add "gcode" sequence to the first command
        YAML::Node gcode = YAML::Node(YAML::NodeType::Sequence);

        for (std::string gcode_line : slicer_gcode_)
        {
            gcode.push_back(gcode_line);
        }

        direct_print_command["command-properties"]["gcode"] = gcode;

        commands.push_back(direct_print_command);
    }


    //Iterate through each of the added parts in the path
    for (size_t part_id : part_addition_order)
    {
        Part::PART_TYPE part_type = initial_assembly_->getPartById(part_id)->getType();

        RCLCPP_DEBUG(logger(), "Adding PLACE_PART with type %d and name %s", static_cast<int>(part_type), initial_assembly_->getPartById(part_id)->getName().c_str());

        //Do nothing with the base object if it's internal  //TODO which it must be at the moment - this must be the first part in the list right?
        if (part_id == base_part_->getId() && base_part_->getType() == Part::INTERNAL)
        {
            RCLCPP_DEBUG(logger(), "Skipping base object");
 
            continue;
        }

        YAML::Node place_part_command;

        if (part_type == Part::SCREW)
            place_part_command["command-type"] = "PLACE_SCREW";

        else
            place_part_command["command-type"] = "PLACE_PART";

        place_part_command["command-properties"]["part-id"] = part_id;

        //If it's an internal part, add the command to vibrate the part first
        if (part_type == Part::INTERNAL)
        {
            YAML::Node vibrate_part_command;

            vibrate_part_command["command-type"] = "VIBRATE_PART";

            vibrate_part_command["command-properties"]["part-id"] = part_id;

            commands.push_back(vibrate_part_command);
        }

        commands.push_back(place_part_command);
    }

    root["commands"] = commands;

    std::ofstream fout(run_output_dir_ + "assembly_plan.yaml");

    std::cout << "Output path: " << run_output_dir_ + "assembly_plan.yaml" << std::endl;

    fout << root;

    fout.close();
}

/*  
    Generate the initial internal part positions by populating the print bed, and generate the initial external part positions using the bay occupancy
*/
void Assembler::generateInitialPartPositions()
{
    RCLCPP_INFO(logger(), "Generating ordered part addition");

    //Arrange the internal parts on the bed
    if (!arrangeInternalParts())
    {
        RCLCPP_WARN(logger(), "Internal parts cannot be arranged on bed — continuing without bed layout");
    }

    // RCLCPP_INFO(logger(), "Setting external part bay positions");

    // for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms())
    // {
    //     if (part->getType() != Part::EXTERNAL) continue;

    //     if (part->hasBayAssigned())
    //     {
    //         // Already assigned by assignExternalBayPositions() — position already
    //         // set in initial_assembly_, nothing to do.
    //     }
    //     else
    //     {
            
    //         initial_assembly_->setUnassembledPart(part, part->generateBayPosition(bay_occupancy_));
    //     }
    // }
}

void Assembler::alignAssemblyPathToInitialAssembly()
{
    RCLCPP_INFO(logger(), "Aligning assembly path with initial assembly base part");

    gp_Pnt target_base_part_pos = target_assembly_->getAssembledPartTransforms()[base_part_]; //TODO need to check this part exists

    gp_Pnt initial_base_part_pos = initial_assembly_->getUnassembledPartTransforms()[base_part_];

    gp_Vec delta(target_base_part_pos, initial_base_part_pos);

    for (std::shared_ptr<AssemblyNode> node : assembly_path_)
    {
        //Unassembled parts get set to initial positions
        for (auto const& [part, transform] : node->assembly_->getUnassembledPartTransforms())
        {
            node->assembly_->setUnassembledPart(part, initial_assembly_->getUnassembledPartTransforms()[part]);
        }

        for (auto const& [part, transform] : node->assembly_->getAssembledPartTransforms())
        {
            node->assembly_->setAssembledPart(part, transform.Translated(delta));
        }
    }

    // Sync target_assembly_ with the translated positions from the last path node.
    // When loading from cache, assembly_path_.back() is a fresh Assembly (not target_assembly_),
    // so target_assembly_ must be updated explicitly. When the DFS generated the path,
    // assembly_path_.back()->assembly_ IS target_assembly_, so this is a no-op.
    if (!assembly_path_.empty() && assembly_path_.back()->assembly_ != target_assembly_)
    {
        for (auto const& [part, transform] : assembly_path_.back()->assembly_->getAssembledPartTransforms())
        {
            target_assembly_->setAssembledPart(part, transform);
        }
    }
}

// /*
//     Align the positions of the assembled parts in the target assembly to the position of the base part as given by the intial assembly
// */
// void Assembler::alignTargetAssemblyToInitialAssembly()
// {
//     RCLCPP_INFO(logger(), "Aligning target assembly with initial assembly base part");
    
//     gp_Pnt target_base_part_pos = target_assembly_->getAssembledPartTransforms()[base_part_]; //TODO need to check this part exists

//     gp_Pnt initial_base_part_pos = assembly_path_[1]->assembly_->getAssembledPartTransforms()[base_part_];

//     RCLCPP_INFO(logger(), "Target base part pos: %f, %f, %f", target_base_part_pos.X(), target_base_part_pos.Y(), target_base_part_pos.Z());

//     RCLCPP_INFO(logger(), "Initial base part pos: %f, %f, %f", initial_base_part_pos.X(), initial_base_part_pos.Y(), initial_base_part_pos.Z());

//     gp_Vec delta(initial_base_part_pos, target_base_part_pos);

//     RCLCPP_INFO(logger(), "Delta: %f, %f, %f", delta.X(), delta.Y(), delta.Z());

//     for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms())  //All parts in target assembly are assemble
//     {
//         target_assembly_->setAssembledPart(part, transform.Translated(delta));
//     }

//     target_base_part_pos = target_assembly_->getAssembledPartTransforms()[base_part_]; //TODO need to check this part exists

//     initial_base_part_pos = assembly_path_[1]->assembly_->getAssembledPartTransforms()[base_part_];

//     RCLCPP_INFO(logger(), "Target base part pos: %f, %f, %f", target_base_part_pos.X(), target_base_part_pos.Y(), target_base_part_pos.Z());

//     RCLCPP_INFO(logger(), "Initial base part pos: %f, %f, %f", initial_base_part_pos.X(), initial_base_part_pos.Y(), initial_base_part_pos.Z());
// }



/* Output: vector of parts ids in the order that they must be assembled to go from starting assembly to target assembly

    Iterates through the assembly states and determines which part must be added to go from one state to the next
*/
std::vector<size_t> Assembler::generatePartAdditionOrder()
{
    RCLCPP_INFO(logger(), "Generating ordered part addition");

    std::vector<size_t> ordered_part_additions;

    for (std::shared_ptr<AssemblyNode> node : assembly_path_)
    {
        for (size_t part_id : node->assembly_->getAssembledPartIds())
        {
            bool part_present = false;

            for (size_t id : ordered_part_additions)
            {
                if (id == part_id)
                    part_present = true;
            }

            if (!part_present)
            {
                ordered_part_additions.push_back(part_id);

                RCLCPP_DEBUG(logger(), "Part %ld", part_id);

                continue;
            }
        }
    }

    return ordered_part_additions;
}

/* Arrange internal parts on the print bed
*/
bool Assembler::arrangeInternalParts()
{
    RCLCPP_INFO(logger(), "Arranging initial internal parts on bed");

    double currentY = PRINT_BED_BOTTOM_LEFT[1];

    double currentX = PRINT_BED_TOP_RIGHT[0];

    double nextY = PRINT_BED_BOTTOM_LEFT[1];

    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms())    //All parts unassembled in initial assembly
    {
        if (part->getType() != Part::INTERNAL)
            continue;

        while (true)
        {
            TopoDS_Shape shape = *part->getShape();

            gp_Pnt part_position(currentX - ShapeAxisSize(shape, 0) / 2, currentY + ShapeAxisSize(shape, 1) / 2, ShapeAxisSize(shape, 2) / 2);

            double nextX = currentX - ShapeAxisSize(shape, 0) - PRINT_MIN_SPACING;

            double topY = currentY + ShapeAxisSize(shape, 1) + PRINT_MIN_SPACING;

            //Check the new position is within parts bay bounds
            if (topY > PRINT_BED_TOP_RIGHT[1])
            {
                //Parts can't fit, return false
                return false;
            }

            else if (nextX < PRINT_BED_BOTTOM_LEFT[0])
            {
                //Start a new y layer, try again with this part
                currentY = nextY;
                currentX = PRINT_BED_TOP_RIGHT[0];

                continue;
            }

            //Otherwise, part fits
            initial_assembly_->setUnassembledPart(part, part_position);

            currentX = nextX;

            nextY = std::max(topY, nextY);
            
            break;
        }
    }

    return true;
}

/* Generate GCode for the internal parts, the print positions of which are given by the initial assembly
*/
void Assembler::generateSlicerGcode()
{
    RCLCPP_INFO(logger(), "Generating Slicer Gcode for internal parts");

    //Find internal parts
    //They will already be in the correct position from previously arranging them
    //Save each as its own stl file

    int i = 0;

    std::vector<std::string> filenames;

    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms()) //No assembled parts in target assembly
    {
        if (part->getType() != Part::INTERNAL)
            continue;

        //Put the part in its correct position before exporting as STL
        part->setCentroidPosition(transform);

        std::stringstream filename_ss;

        filename_ss << run_output_dir_ << "internal_part_" << i << ".stl";

        std::stringstream filename_local_ss;

        filename_local_ss << "internal_part_" << i << ".stl";

        part->saveShape(filename_ss.str());

        filenames.push_back(filename_local_ss.str());

        i ++;        
    }

    //Call Prusa Slicer with the stl files and the correct settings (don't allow rearranging)

    std::stringstream command_ss;

    // Use an absolute path for the config so it's still found after `cd run_output_dir_`.
    const std::string config_dir = slicer_config_dir_.empty() ? WORKING_DIR : slicer_config_dir_;
    const std::string abs_config =
        std::filesystem::absolute(config_dir + "arms_prusa_config.ini").string();

    if (!std::filesystem::exists(abs_config)) {
        RCLCPP_WARN(logger(), "Slicer config not found: '%s' — skipping gcode generation. "
                    "Pass --slicer-config-dir (arms-plan) or set slicer_config_dir_ to the "
                    "directory containing arms_prusa_config.ini.",
                    abs_config.c_str());
        return;
    }

    command_ss << "(cd " << run_output_dir_ << " && "
               << "prusa-slicer --export-gcode --dont-arrange --merge "
               << "--output assembler.gcode --load " << abs_config;
    //command_ss << "(cd " << WORKING_DIR << " && " << "slic3r --dont-arrange --merge --output assembler.gcode --load arms_prusa_config.ini";

    for (std::string filename : filenames)
        command_ss << " " << filename;

    command_ss << ")";

    std::cout << "Slicing command: " << command_ss.str() << std::endl;

    int ret = std::system(command_ss.str().c_str());

    if (ret == 0) {
        std::cout << "Slicing completed successfully!" << std::endl;
    } else {
        std::cerr << "Error: PrusaSlicer execution failed!" << std::endl;
    }

    std::stringstream gcode_ss;

    gcode_ss << run_output_dir_ << "assembler.gcode";

    //Load the GCode that Prusa writes
    std::ifstream gcodeFile(gcode_ss.str());
    if (!gcodeFile)
        throw std::runtime_error("Assembler: cannot open PrusaSlicer output: " + gcode_ss.str());

    std::string line;
    while (std::getline(gcodeFile, line)) 
    {
        slicer_gcode_.push_back(line);

        // if (line == "; Filament-specific end gcode")
        //     break;
    }

    //Iterate through gcode in reverse and delete up to and including the END_PRINT macro
    for (int i = slicer_gcode_.size() - 1; i >= 0; i--)
    {
        if (slicer_gcode_[i] == "END_PRINT")
        {
            slicer_gcode_.pop_back();
            break;   
        }

        else
        {
            slicer_gcode_.pop_back();
        }
    }
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::depthFirstZAssembly()
{
    RCLCPP_INFO(logger(), "Generating assembly path (DFS)");

    std::vector<std::shared_ptr<AssemblyNode>> path;

    std::shared_ptr<AssemblyNode> target_node;

    std::shared_ptr<AssemblyNode> base_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());
    base_node->assembly_ = target_assembly_;
    base_node->id_ = nodeIdGenerator(target_assembly_->getAssembledPartIds());

    // Compute assembly center (centroid of all assembled part positions in the target assembly)
    gp_Pnt assembly_center(0, 0, 0);
    {
        auto target_transforms = target_assembly_->getAssembledPartTransforms();
        if (!target_transforms.empty()) {
            double sum_x = 0, sum_y = 0, sum_z = 0;
            for (auto const& [p, t] : target_transforms) {
                sum_x += t.X();
                sum_y += t.Y();
                sum_z += t.Z();
            }
            int n = target_transforms.size();
            assembly_center = gp_Pnt(sum_x / n, sum_y / n, sum_z / n);
        }
    }

    // DFS uses a stack (LIFO)
    std::stack<std::shared_ptr<AssemblyNode>> stack;

    std::set<std::shared_ptr<AssemblyNode>> visited_nodes;
    std::map<std::shared_ptr<AssemblyNode>, std::shared_ptr<AssemblyNode>> parents;

    stack.push(base_node);
    visited_nodes.emplace(base_node);

    while (!stack.empty())
    {
        std::shared_ptr<AssemblyNode> current_node = stack.top();
        stack.pop();

        RCLCPP_INFO(logger(), "New node. Assembled parts:");

        for (auto part_position : current_node->assembly_->getAssembledPartTransforms())
        {
            RCLCPP_INFO(logger(), "Name %s, ID %ld, type %d", part_position.first->getName().c_str(), part_position.first->getId(), part_position.first->getType());
        }

        // Target condition: no assembled parts remaining
        if (current_node->assembly_->getAssembledPartTransforms().empty())
        {
            target_node = current_node;
            break; // DFS typically stops when first target found
        }

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        // Precompute the distance of each neighbour's newly-unassembled part from the assembly center
        auto get_unassembled_distance = [&](const std::shared_ptr<AssemblyNode>& neighbour) -> double {
            auto current_assembled = current_node->assembly_->getAssembledPartTransforms();
            auto neighbour_assembled = neighbour->assembly_->getAssembledPartTransforms();
            for (auto const& [part, transform] : current_assembled) {
                if (neighbour_assembled.count(part) == 0) {
                    double dx = transform.X() - assembly_center.X();
                    double dy = transform.Y() - assembly_center.Y();
                    double dz = transform.Z() - assembly_center.Z();
                    return std::sqrt(dx*dx + dy*dy + dz*dz);
                }
            }
            return 0.0;
        };

        // Sort ascending so the furthest part ends up pushed last (on top of LIFO stack, explored first)
        std::sort(neighbours.begin(), neighbours.end(),
            [&](const std::shared_ptr<AssemblyNode>& a, const std::shared_ptr<AssemblyNode>& b) {
                return get_unassembled_distance(a) < get_unassembled_distance(b);
            });

        for (const std::shared_ptr<AssemblyNode>& neighbour : neighbours)
        {
            if (visited_nodes.find(neighbour) != visited_nodes.end())
                continue;

            visited_nodes.emplace(neighbour);
            parents[neighbour] = current_node;
            stack.push(neighbour);
        }
    }

    // If we never found a target, or target has no parent (and isn't base) -> fail
    if (!target_node || (target_node != base_node && !parents.count(target_node)))
    {
        RCLCPP_WARN(logger(), "No assembly path found (DFS) — check /assembler/grasp_debug markers for rejection reasons");
        return path;
    }

    // Reconstruct path (same as your BFS)
    std::shared_ptr<AssemblyNode> path_node = target_node;
    path.push_back(path_node);

    while (parents.count(path_node))
    {
        path_node = parents[path_node];
        path.push_back(path_node);
    }

    RCLCPP_INFO(logger(), "Assembly path found (DFS)");
    return path;
}


std::vector<std::shared_ptr<AssemblyNode>> Assembler::breadthFirstZAssembly()
{
    RCLCPP_INFO(logger(), "Generating assembly path");

    std::vector<std::shared_ptr<AssemblyNode>> path;

    std::shared_ptr<AssemblyNode> target_node;

    std::shared_ptr<AssemblyNode> first_part_node;

    std::shared_ptr<AssemblyNode> base_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

    base_node->assembly_ = target_assembly_;

    base_node->id_ = nodeIdGenerator(target_assembly_->getAssembledPartIds());

    std::queue<std::shared_ptr<AssemblyNode>> queue;

    std::set<std::shared_ptr<AssemblyNode>> visited_nodes;

    std::map<std::shared_ptr<AssemblyNode>, std::shared_ptr<AssemblyNode>> parents;

    queue.push(base_node);

    visited_nodes.emplace(base_node);

    while (queue.size() > 0)
    {
        std::shared_ptr<AssemblyNode> current_node = queue.front();
 
        queue.pop();

        //Check if current node is target node (e.g., has no assembled parts)
        if (current_node->assembly_->getAssembledPartTransforms().size() == 0)               
            target_node = current_node;

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        for (std::shared_ptr<AssemblyNode> neighbour : neighbours)
        {
            //If neighbour has already been visited, move on
            if (visited_nodes.find(neighbour) != visited_nodes.end())
                continue;

            visited_nodes.emplace(neighbour);

            queue.push(neighbour);

            parents[neighbour] = current_node;
        }
    }

    if (!parents.count(target_node))
    {
        RCLCPP_FATAL(logger(), "No assembly path found");
        return path;  // caller checks path.empty() and handles gracefully
    }

    std::shared_ptr<AssemblyNode> path_node = target_node;

    path.push_back(target_node);

    while (parents.count(path_node))
    {
        path_node = parents[path_node];

        path.push_back(path_node);
    }

    //TODO - find internal first node

    RCLCPP_INFO(logger(), "Assembly path found");

    return path;
}

// Check whether `part` can be removed (i.e. placed last) in the current DFS
// node.  All parts in `assembled` have their OCCT shapes at their assembly
// centroid positions (setPartTransforms was called by the caller).
//
// Returns nullopt if infeasible; otherwise the grasp in local frame (mm) —
// zero for non-external parts.
std::optional<gp_Pnt> Assembler::edge_feasible(
    std::shared_ptr<Part> part,
    const PartTransformMap& assembled)
{
    if (part->getType() == Part::SCREW)  return gp_Pnt(0,0,0);
    if (part->isPushfit())               return gp_Pnt(0,0,0);
    if (!part->get_mesh_asset())         return gp_Pnt(0,0,0);
    if (!collision_adapter_)             return gp_Pnt(0,0,0);

    const std::string part_id = part->getName() + "_" + std::to_string(part->getId());
    const gp_Pnt assembly_centroid = assembled.at(part);  // mm

    // ---- z-step lift collision check (all parts) ----
    // Skipped entirely when the user has set a non-zero collision volume threshold,
    // which signals deliberately generous collision handling.
    if (collision_volume_threshold_ == 0.0)
    {
        const double STEP_MM = 1.0;
        const int    N_STEPS = 5;

        auto part_lf_shape = std::make_shared<TopoDS_Shape>(
            LocalFrameShapeM(*part->getShape()));

        bool z_ok = true;
        for (int step = 1; step <= N_STEPS && z_ok; ++step)
        {
            gp_Trsf lifted;
            lifted.SetTranslation(gp_Vec(
                assembly_centroid.X() * 0.001,
                assembly_centroid.Y() * 0.001,
                (assembly_centroid.Z() + step * STEP_MM) * 0.001));
            collision_adapter_->add_or_update(part_id, part_lf_shape, lifted);

            for (auto const& [other, _] : assembled)
            {
                if (other->getId() == part->getId()) continue;
                if (!other->getShape()) continue;
                const std::string other_id = other->getName() + "_" + std::to_string(other->getId());
                if (!collision_adapter_->collision_free(part_id, other_id))
                {
                    z_ok = false;
                    break;
                }
            }
        }

        // Restore the part to its original assembly pose in the adapter.
        {
            gp_Trsf original;
            original.SetTranslation(gp_Vec(
                assembly_centroid.X() * 0.001,
                assembly_centroid.Y() * 0.001,
                assembly_centroid.Z() * 0.001));
            collision_adapter_->add_or_update(part_id, part_lf_shape, original);
        }

        if (!z_ok) return std::nullopt;
    }

    // Non-external parts: z-step is sufficient.
    if (part->getType() != Part::EXTERNAL) return gp_Pnt(0, 0, 0);
    if (!generate_grasps_)                 return gp_Pnt(0, 0, 0);
    if (!nozzle_shape_)                    return gp_Pnt(0, 0, 0);

    // ---- Grasp search (external parts) ----
    // VacuumGraspGenerator checks nozzle vs part body AND vs assembled parts.
    std::vector<std::string> assembled_ids;
    assembled_ids.reserve(assembled.size());
    for (auto const& [other, _] : assembled)
    {
        if (other->getId() == part->getId()) continue;
        assembled_ids.push_back(other->getName() + "_" + std::to_string(other->getId()));
    }

    auto grasp_opt = VacuumGraspGenerator::generate(
        part, *collision_adapter_, nozzle_shape_, assembled_ids);

    if (!grasp_opt) return std::nullopt;
    const gp_Pnt grasp = *grasp_opt;

    return grasp;
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::findNodeNeighbours(std::shared_ptr<AssemblyNode> node)
{
    // Set each part to its position within the assembly
    node->assembly_->setPartTransforms();

    std::vector<std::shared_ptr<AssemblyNode>> neighbours;

    const auto& assembled = node->assembly_->getAssembledPartTransforms();

    // Internal (base) parts must be assembled first, so they may only be
    // disassembled when no external parts remain.
    bool has_external = false;
    for (auto const& [p, _] : assembled)
        if (p->getType() == Part::EXTERNAL) { has_external = true; break; }

    for (auto const& [part, transform] : assembled)
    {
        if (part->getType() == Part::INTERNAL && has_external)
            continue;

        auto grasp_result = edge_feasible(part, assembled);
        if (!grasp_result)
            continue;

        //Create new assembly node
        std::shared_ptr<Assembly> neighbour_assembly = std::shared_ptr<Assembly>(new Assembly());
        std::shared_ptr<AssemblyNode> neighbour_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

        for (auto const& [part_to_add, transform_to_add] : node->assembly_->getUnassembledPartTransforms())
        {
            neighbour_assembly->setUnassembledPart(part_to_add, transform_to_add);
        }

        for (auto const& [part_to_add, transform_to_add] : node->assembly_->getAssembledPartTransforms())
        {
            if (part_to_add->getId() != part->getId())
                neighbour_assembly->setAssembledPart(part_to_add, transform_to_add);
            else
                neighbour_assembly->setUnassembledPart(part_to_add, initial_assembly_->getUnassembledPartTransforms()[part_to_add]);
        }

        neighbour_node->assembly_   = neighbour_assembly;
        neighbour_node->id_         = nodeIdGenerator(neighbour_assembly->getAssembledPartIds());
        // Store the part placed (and its grasp) to reach this neighbour from the current node.
        neighbour_node->edge_part_  = part;
        neighbour_node->edge_grasp_ = *grasp_result;

        neighbours.push_back(neighbour_node);
    }

    return neighbours;
}

size_t Assembler::nodeIdGenerator(std::vector<size_t> object_ids)
{
    for (auto const& [id, obj_ids] : node_id_map_)
    {
        if (object_ids.size() != obj_ids.size())
            continue;
        
        bool allFound = true;

        for (size_t obj_id_1 : object_ids)
        {
            bool found = false;

            for (size_t obj_id_2 : obj_ids)
            {
                if (obj_id_1 == obj_id_2)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                allFound = false;

                break;
            }
        }

        if (allFound)
            return id;
    }

    //Not found, new ID
    node_id_map_[next_node_ID_++] = object_ids;

    return next_node_ID_ - 1;
}

/* Create a new set assembly state, for now just copying over the part positions
*/
void Assembler::generateInitialAssembly()
{
    RCLCPP_INFO(logger(), "Generating initial assembly");

    initial_assembly_ = std::shared_ptr<Assembly>(new Assembly());

    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms()) //No unassembled parts in target assembly
    {
        initial_assembly_->setUnassembledPart(part, transform);
    }

    // Pre-assign bay positions for external parts so they are available during DFS.
    assignExternalBayPositions();

    // Move shapes to their assembly positions — OCCT face queries inside
    // VacuumGraspGenerator must be consistent during DFS.
    target_assembly_->setPartTransforms();

    // Load the vacuum nozzle once for use in edge_feasible and generate().
    // Try to load from the user-supplied STEP file first; fall back to a
    // 4.2 mm radius × 20 mm cylinder if the file is absent or unreadable.
    // raw_nozzle keeps the mm-space shape before local-frame conversion.
    TopoDS_Shape raw_nozzle;
    if (!tool_config_.mesh_file.empty()) {
        STEPControl_Reader reader;
        if (reader.ReadFile(tool_config_.mesh_file.c_str()) == IFSelect_RetDone) {
            reader.TransferRoots();
            TopoDS_Shape shape = reader.OneShape();
            if (!shape.IsNull()) {
                nozzle_mesh_ = Tessellator::tessellate(shape, MESH_DEFLECTION_MM);
                raw_nozzle   = shape;
                RCLCPP_INFO(logger(), "Nozzle mesh loaded from '%s': %zu verts, %zu tris",
                            tool_config_.mesh_file.c_str(),
                            nozzle_mesh_->vertices.size(),
                            nozzle_mesh_->triangles.size());
            }
        }
        if (!nozzle_mesh_)
            RCLCPP_WARN(logger(), "Failed to load nozzle mesh from '%s' — using cylinder fallback",
                        tool_config_.mesh_file.c_str());
    }
    if (!nozzle_mesh_) {
        // Fallback: radius 4.2 mm, height 20 mm, bottom at z=0, centroid at z=10.
        raw_nozzle  = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), 4.2, 20.0).Shape();
        nozzle_mesh_ = Tessellator::tessellate(raw_nozzle, MESH_DEFLECTION_MM);
    }

    // Convert nozzle to local-frame metres for collision (same convention as
    // Tessellator: bbox centroid at origin, mm → m).
    nozzle_shape_ = std::make_shared<TopoDS_Shape>(LocalFrameShapeM(raw_nozzle));

    // Align the nozzle so its contact face (min-z) sits at exactly z = -0.01 m.
    // VacuumGraspGenerator places the nozzle origin at face_z + GAP + NOZZLE_H_HALF
    // and expects the bottom to be 0.01 m below that origin.  The cylinder satisfies
    // this by construction; loaded STEP geometry may not.
    {
        double min_z = std::numeric_limits<double>::max();
        for (const auto& v : nozzle_mesh_->vertices)
            min_z = std::min(min_z, v[2]);
        const double expected_bottom_m = -10.0 * 0.001;
        const double shift = expected_bottom_m - min_z;
        if (std::abs(shift) > 1e-6) {
            for (auto& v : nozzle_mesh_->vertices)
                v[2] += shift;
            gp_Trsf z_adj;
            z_adj.SetTranslation(gp_Vec(0.0, 0.0, shift));
            *nozzle_shape_ = BRepBuilderAPI_Transform(*nozzle_shape_, z_adj, true).Shape();
            RCLCPP_INFO(logger(), "Nozzle z-shifted %.2f mm to align contact face",
                        shift * 1000.0);
        }
    }

    // Apply the per-tool XYZ correction from the tool config (mm → m).
    // This corrects any lateral offset visible in the visualisation.
    {
        const double dx = tool_config_.offset_x_mm * 0.001;
        const double dy = tool_config_.offset_y_mm * 0.001;
        const double dz = tool_config_.offset_z_mm * 0.001;
        if (std::abs(dx) > 1e-9 || std::abs(dy) > 1e-9 || std::abs(dz) > 1e-9) {
            for (auto& v : nozzle_mesh_->vertices) {
                v[0] += dx;
                v[1] += dy;
                v[2] += dz;
            }
            gp_Trsf xyz_adj;
            xyz_adj.SetTranslation(gp_Vec(dx, dy, dz));
            *nozzle_shape_ = BRepBuilderAPI_Transform(*nozzle_shape_, xyz_adj, true).Shape();
            RCLCPP_INFO(logger(), "Nozzle offset applied: (%.2f, %.2f, %.2f) mm",
                        tool_config_.offset_x_mm,
                        tool_config_.offset_y_mm,
                        tool_config_.offset_z_mm);
        }
    }

    // Build the neutral scene model from the target assembly (all parts present
    // at their assembled positions).  Poses are in metres (mm * 0.001).
    scene_.clear();
    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms())
    {
        if (!part->getShape()) continue;

        gp_Trsf pose;
        pose.SetTranslation(gp_Vec(transform.X() * 0.001,
                                   transform.Y() * 0.001,
                                   transform.Z() * 0.001));

        const std::string id = part->getName() + "_" + std::to_string(part->getId());
        auto lf_shape = std::make_shared<TopoDS_Shape>(LocalFrameShapeM(*part->getShape()));
        scene_.add_object(id, part->get_mesh_asset(), lf_shape, SceneRole::Part, pose, true);
    }

    const double safety_margin_m = MESH_DEFLECTION_MM * 0.5 * 0.001;
    collision_adapter_ = std::make_unique<OcctCollisionAdapter>(safety_margin_m);
    collision_adapter_->sync(scene_);

    RCLCPP_INFO(logger(), "SceneModel built with %zu objects (OCCT, safety margin %.4f mm)",
                scene_.present_object_ids().size(),
                safety_margin_m * 1000.0);
}

void Assembler::assignExternalBayPositions()
{
    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms())
    {
        if (part->getType() != Part::EXTERNAL) continue;
        if (!part->hasBayAssigned())
            initial_assembly_->setUnassembledPart(
                part, part->generateBayPosition(bay_occupancy_));
    }
}

/* Create printable jigs for each external part to fit int he parts bay. Also allocates each external
part to a bay in the parts bay and sets the transforms of the parts in the initial assembly
*/
void Assembler::generateNegatives()
{
    RCLCPP_INFO(logger(), "Generating external part jigs");

    // Move OCC shapes to their bay XY positions before generating jigs.
    // Then explicitly set z = JIG_HEIGHT/2 for every external part so all jig
    // STLs end up with their bottom face at z=0 (consistent across all parts).
    initial_assembly_->setPartTransforms();

    const auto& unassembled = initial_assembly_->getUnassembledPartTransforms();
    RCLCPP_INFO(logger(), "  %zu unassembled parts in initial assembly", unassembled.size());

    for (auto const& [part, transform] : unassembled)
    {
        RCLCPP_INFO(logger(), "  Part: %s  type=%d  bay_assigned=%d",
                    part->getName().c_str(), part->getType(), (int)part->hasBayAssigned());

        //Only create negatives or external parts
        if (part->getType() != Part::EXTERNAL)
            continue;

        part->setCentroidPosition(gp_Pnt(transform.X(), transform.Y(), JIG_CENTER_Z));

        CradleGenerator cradle_gen(part->getName(), *part->getShape(), cradle_scaling_distance_);

        float part_jig_z_offset = cradle_gen.createSimpleNegative(BAY_SIZES[part->getBaySizeIndex()], part->getBayIndex(), run_output_dir_);

        RCLCPP_INFO(logger(), "Jig z offset: %f", part_jig_z_offset);

        RCLCPP_INFO(logger(), "Setting part transform %f %f %f", transform.X(), transform.Y(), JIG_CENTER_Z + part_jig_z_offset);

        initial_assembly_->setUnassembledPart(part, gp_Pnt(transform.X(), transform.Y(), JIG_CENTER_Z + part_jig_z_offset));

    }
}

/* Generate vacuum grasps for all external parts, using the full assembly as
   the collision context.  Called in non-path mode (generate_path_=false).
   In path mode, grasps are found per-edge during DFS and propagated via
   edge_part_/edge_grasp_ fields on AssemblyNode — this function is skipped.
*/
void Assembler::generateGrasps()
{
    if (!collision_adapter_ || !nozzle_shape_) return;

    // Ensure shapes are at assembly positions for consistent OCCT queries.
    target_assembly_->setPartTransforms();

    const auto& all_parts = target_assembly_->getAssembledPartTransforms();

    for (auto const& [part, transform] : all_parts)
    {
        // (cancellation hook — currently a no-op; add std::atomic<bool>* flag if needed)

        if (part->getType() != Part::EXTERNAL) continue;

        // Build scene IDs for every other assembled part.
        std::vector<std::string> assembled_ids;
        assembled_ids.reserve(all_parts.size() - 1);
        for (auto const& [other, _] : all_parts)
        {
            if (other->getId() == part->getId()) continue;
            assembled_ids.push_back(other->getName() + "_" + std::to_string(other->getId()));
        }

        auto grasp = VacuumGraspGenerator::generate(
            part, *collision_adapter_, nozzle_shape_, assembled_ids);

        if (grasp)
            part->setVacuumGrasp(*grasp);
        else
            RCLCPP_WARN(logger(), "generateGrasps: no grasp found for %s",
                        part->getName().c_str());
    }
}

std::vector<GraspAttempt> Assembler::debugGrasps()
{
    std::vector<GraspAttempt> all_attempts;
    if (!collision_adapter_ || !nozzle_shape_ || !target_assembly_) return all_attempts;

    target_assembly_->setPartTransforms();

    const auto& all_parts = target_assembly_->getAssembledPartTransforms();

    for (auto const& [part, transform] : all_parts)
    {
        if (part->getType() != Part::EXTERNAL) continue;

        std::vector<std::string> assembled_ids;
        assembled_ids.reserve(all_parts.size() - 1);
        for (auto const& [other, _] : all_parts)
        {
            if (other->getId() == part->getId()) continue;
            assembled_ids.push_back(other->getName() + "_" + std::to_string(other->getId()));
        }

        VacuumGraspGenerator::generate(
            part, *collision_adapter_, nozzle_shape_, assembled_ids, &all_attempts);
    }

    return all_attempts;
}