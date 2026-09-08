#include "assembler/Assembler.hpp"
#include "assembler/CoalAdapter.hpp"
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
#include <algorithm>

#include <atomic>
#include <mutex>
#include <thread>

#include <BRepMesh_IncrementalMesh.hxx>

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

// One search event, as a JSON object on its own line.  It rides the normal
// stdout stream the viewer already reads, so no extra channel is needed.
void Assembler::traceDfs(const std::string& json) const
{
    if (!trace_dfs_) return;
    std::printf("[dfs] %s\n", json.c_str());
    std::fflush(stdout);
}

// Minimal JSON string escaping — part names carry quotes and backslashes.
static std::string jesc(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c == '\n' || c == '\r' || c == '\t') out += ' ';
        else out += c;
    }
    return out;
}

static std::string idListJson(const PartTransformMap& parts)
{
    std::string s = "[";
    bool first = true;
    for (auto const& [p, _] : parts) {
        if (!first) s += ",";
        s += std::to_string(p->getId());
        first = false;
    }
    return s + "]";
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
    splits_used_ = 0;
    split_attempts_ = 0;

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

        // Reconcile the part list with any split taken on the chosen path.  A
        // part that was cut is superseded by its two pieces: the finished
        // assembly contains the pieces, not the original, and the original must
        // not also be printed.  base_node->assembly_ IS target_assembly_, so
        // rewriting it here also fixes the first node of the path.
        for (const auto& node : assembly_path_)
        {
            if (!node->is_split_step_ || !node->split_source_) continue;

            RCLCPP_INFO(logger(),
                        "Replacing %s with its two pieces (%s, %s) in the build",
                        node->split_source_->getName().c_str(),
                        node->split_lower_->getName().c_str(),
                        node->split_upper_->getName().c_str());

            target_assembly_->removeAssembledPart(node->split_source_);
            target_assembly_->setAssembledPart(
                node->split_lower_, ShapeCentroid(*node->split_lower_->getShape()));
            target_assembly_->setAssembledPart(
                node->split_upper_, ShapeCentroid(*node->split_upper_->getShape()));

            // The source stays in initial_assembly_ on purpose.  A cut piece is a
            // sequencing concept, not a printing one: the printer still lays down
            // one continuous object, and it is the *g-code* that gets divided at
            // the cut height.  Slicing the pieces separately would ask PrusaSlicer
            // to slice a body floating 15 mm above the bed.
            splits_.push_back({node->split_source_, node->split_lower_,
                               node->split_upper_, node->edge_part_, node->split_z_rel_});
        }

        // Pieces created while exploring branches that were abandoned would
        // otherwise still be printed.
        {
            std::set<size_t> on_path;
            for (const auto& node : assembly_path_)
                for (auto const& [p, _] : node->assembly_->getUnassembledPartTransforms())
                    on_path.insert(p->getId());
            for (auto const& [p, _] : target_assembly_->getAssembledPartTransforms())
                on_path.insert(p->getId());
            // A split source is absent from the finished assembly — its pieces are
            // there instead — but it is still the object the printer lays down, so
            // it must survive this pass.
            for (auto const& sp : splits_)
                on_path.insert(sp.source->getId());

            std::vector<std::shared_ptr<Part>> drop;
            for (auto const& [p, _] : initial_assembly_->getUnassembledPartTransforms())
                if (!on_path.count(p->getId())) drop.push_back(p);
            for (const auto& p : drop) {
                RCLCPP_INFO(logger(), "Dropping unused part %s (id %ld)",
                            p->getName().c_str(), p->getId());
                initial_assembly_->removeUnassembledPart(p);
            }
        }

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
    // Which print segment resumes after each mid-build insertion.
    std::map<size_t, size_t> print_segment_after_part;
    for (size_t i = 0; i < splits_.size(); ++i)
        if (splits_[i].freed)
            print_segment_after_part[splits_[i].freed->getId()] = i + 1;

    auto segments = slicer_gcode_segments_;
    if (segments.empty()) segments.push_back(slicer_gcode_);

    GCodeGenerator::generate(initial_assembly_, target_assembly_, base_part_,
                             part_addition_order, segments,
                             print_segment_after_part, run_output_dir_);
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

        gp_Vec pick_position = SumPoints(initial_transform, part->getGraspOffset());

        gp_Vec place_position = SumPoints(target_assembly_->getAssembledPartTransforms()[part], part->getGraspOffset());

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
        // Which end effector this part is picked with, so the machine knows
        // whether the angle/width fields below are meaningful.
        const char* tool_name = "none";
        switch (part->getGraspTool()) {
            case Part::GraspTool::VACUUM: tool_name = "vacuum";  break;
            case Part::GraspTool::PPG:    tool_name = "gripper"; break;
            case Part::GraspTool::NONE:   tool_name = "none";    break;
        }
        designate_part_command["command-properties"]["grasp-tool"] = std::string(tool_name);

        commands.push_back(designate_part_command);
    }

    //Designate external parts
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {        
        if (part->getType() != Part::EXTERNAL)
            continue;

        gp_Vec pick_position = SumPoints(initial_transform, part->getGraspOffset());

        gp_Vec place_position = SumPoints(target_assembly_->getAssembledPartTransforms()[part], part->getGraspOffset());

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
        // Reported for whichever tool is in use, matching the pick/place above.
        designate_part_command["command-properties"]["grasp-pos-x"] = part->getGraspOffset().X();
        designate_part_command["command-properties"]["grasp-pos-y"] = part->getGraspOffset().Y();
        designate_part_command["command-properties"]["grasp-pos-z"] = part->getGraspOffset().Z();

        // Which end effector picks this part, and — for the gripper — the jaw
        // pose.  The angle/width fields are emitted only when they mean something;
        // they were previously written for every part from an uninitialised
        // PPGGrasp, which produced convincing-looking but meaningless numbers.
        const char* ext_tool = "none";
        switch (part->getGraspTool()) {
            case Part::GraspTool::VACUUM: ext_tool = "vacuum";  break;
            case Part::GraspTool::PPG:    ext_tool = "gripper"; break;
            case Part::GraspTool::NONE:   ext_tool = "none";    break;
        }
        designate_part_command["command-properties"]["grasp-tool"] = std::string(ext_tool);

        if (part->getGraspTool() == Part::GraspTool::PPG)
        {
            const PPGGrasp& g = part->getPPGGrasp();
            designate_part_command["command-properties"]["part-grasp-angle"] = g.rotation_;
            designate_part_command["command-properties"]["part-grasp-width"] = g.width_;
            designate_part_command["command-properties"]["part-grasp-jaw-z"] = g.jaw_z_;
        }

        commands.push_back(designate_part_command);
    }

    // Designate the print phases.  A split part is printed in two goes with the
    // freed part dropped in between; the machine needs to know where the print
    // pauses and which part goes in at that point.
    for (size_t i = 0; i < splits_.size(); ++i)
    {
        const auto& sp = splits_[i];

        YAML::Node print_phase;
        print_phase["command-type"] = "DESIGNATE_PRINT_PHASE";
        print_phase["command-properties"]["phase-index"]      = static_cast<int>(i + 1);
        print_phase["command-properties"]["print-segment"]    = static_cast<int>(i + 1);
        print_phase["command-properties"]["source-part-id"]   = sp.source->getId();
        print_phase["command-properties"]["source-part-name"] = sp.source->getName();
        print_phase["command-properties"]["cut-height"]       = sp.z_cut_rel;
        if (sp.freed)
        {
            print_phase["command-properties"]["insert-part-id"]   = sp.freed->getId();
            print_phase["command-properties"]["insert-part-name"] = sp.freed->getName();
        }
        commands.push_back(print_phase);
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

    // Recorded grasp attempts are world-frame positions captured before this
    // translation, so they must move with the assembly — otherwise the viewer
    // draws them at the pre-alignment location while the parts sit on the bed.
    for (auto& a : grasp_attempts_)
    {
        a.x_mm += delta.X();
        a.y_mm += delta.Y();
        a.z_mm += delta.Z();
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

    // Inset by PRINT_MIN_SPACING so the first part in each row isn't flush with
    // the bed edge — the slicer's skirt/brim extends a few mm beyond the part
    // and would otherwise be generated off the bed.
    const double bed_min_x = PRINT_BED_BOTTOM_LEFT[0] + PRINT_MIN_SPACING;
    const double bed_max_x = PRINT_BED_TOP_RIGHT[0]   - PRINT_MIN_SPACING;
    const double bed_min_y = PRINT_BED_BOTTOM_LEFT[1] + PRINT_MIN_SPACING;
    const double bed_max_y = PRINT_BED_TOP_RIGHT[1]   - PRINT_MIN_SPACING;

    double currentY = bed_min_y;

    double currentX = bed_max_x;

    double nextY = bed_min_y;

    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms())    //All parts unassembled in initial assembly
    {
        if (part->getType() != Part::INTERNAL)
            continue;
        if (isSplitPiece(part))   // printed as part of its source object
            continue;

        while (true)
        {
            TopoDS_Shape shape = *part->getShape();

            gp_Pnt part_position(currentX - ShapeAxisSize(shape, 0) / 2, currentY + ShapeAxisSize(shape, 1) / 2, ShapeAxisSize(shape, 2) / 2);

            double nextX = currentX - ShapeAxisSize(shape, 0) - PRINT_MIN_SPACING;

            double topY = currentY + ShapeAxisSize(shape, 1) + PRINT_MIN_SPACING;

            //Check the new position is within parts bay bounds
            if (topY > bed_max_y)
            {
                //Parts can't fit, return false
                return false;
            }

            else if (nextX < bed_min_x)
            {
                //Start a new y layer, try again with this part
                currentY = nextY;
                currentX = bed_max_x;

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
// Divide slicer output at the given heights, in ascending order.
//
// PrusaSlicer brackets every layer with ";LAYER_CHANGE" followed by ";Z:<top>",
// which is the only reliable place to cut: everything up to that marker is a
// finished layer, so each segment is self-contained and the resumed stream picks
// up exactly where it left off.  A layer whose top is at or below the cut belongs
// to the lower segment; the first layer above it starts the next one.
//
// This is why the part is sliced whole rather than as two bodies — the extrusion
// paths are identical to an uninterrupted print, and PrusaSlicer is never asked
// to slice geometry floating above the bed.
std::vector<std::vector<std::string>> Assembler::splitGcodeAtHeights(
    const std::vector<std::string>& gcode,
    std::vector<double>             heights)
{
    std::sort(heights.begin(), heights.end());

    std::vector<std::vector<std::string>> segments;
    segments.emplace_back();

    size_t next_cut = 0;

    for (size_t i = 0; i < gcode.size(); ++i)
    {
        if (next_cut < heights.size() && gcode[i] == ";LAYER_CHANGE")
        {
            // The layer's top height is on the following ";Z:" line.
            double layer_z = -1.0;
            for (size_t j = i + 1; j < gcode.size() && j < i + 4; ++j)
            {
                if (gcode[j].rfind(";Z:", 0) == 0)
                {
                    try { layer_z = std::stod(gcode[j].substr(3)); } catch (...) {}
                    break;
                }
            }

            // First layer sitting above the cut — start a new segment here.
            if (layer_z > heights[next_cut] + 1e-9)
            {
                RCLCPP_INFO(logger(),
                            "Print segment boundary at Z=%.2f mm (cut requested at %.2f mm)",
                            layer_z, heights[next_cut]);
                segments.emplace_back();
                ++next_cut;
            }
        }

        segments.back().push_back(gcode[i]);
    }

    if (next_cut < heights.size())
        RCLCPP_WARN(logger(),
                    "Only %zu of %zu print cuts found — the part may be shorter than "
                    "the requested cut height", next_cut, heights.size());

    return segments;
}

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
        if (isSplitPiece(part))   // its geometry is already in the source STL
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

    // Divide the print wherever a part has to be dropped in mid-build.
    std::vector<double> cuts;
    for (auto const& sp : splits_) cuts.push_back(sp.z_cut_rel);

    slicer_gcode_segments_ = splitGcodeAtHeights(slicer_gcode_, cuts);

    if (slicer_gcode_segments_.size() > 1)
        RCLCPP_INFO(logger(), "Print divided into %zu segments for %zu mid-print insertion(s)",
                    slicer_gcode_segments_.size(), splits_.size());
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::depthFirstZAssembly()
{
    RCLCPP_INFO(logger(), "Generating assembly path (DFS)");

    {
        std::string legend = "{\"ev\":\"parts\",\"map\":{";
        bool first = true;
        for (auto const& [p, _] : target_assembly_->getAssembledPartTransforms()) {
            if (!first) legend += ",";
            legend += "\"" + std::to_string(p->getId()) + "\":{\"name\":\"" +
                      jesc(p->getName()) + "\",\"type\":" +
                      std::to_string(static_cast<int>(p->getType())) + "}";
            first = false;
        }
        traceDfs(legend + "}}");
    }

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

        traceDfs("{\"ev\":\"visit\",\"id\":" + std::to_string(current_node->id_) +
                 ",\"parts\":" + idListJson(current_node->assembly_->getAssembledPartTransforms()) + "}");

        // Target condition: no assembled parts remaining
        if (current_node->assembly_->getAssembledPartTransforms().empty())
        {
            traceDfs("{\"ev\":\"goal\",\"id\":" + std::to_string(current_node->id_) + "}");
            target_node = current_node;
            break; // DFS typically stops when first target found
        }

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        // Splitting a printed part is a last resort: only once plain removal
        // offers nowhere to go from this state.
        if (neighbours.empty())
            neighbours = findSplitNeighbours(current_node);

        if (neighbours.empty())
            traceDfs("{\"ev\":\"dead\",\"id\":" + std::to_string(current_node->id_) + "}");

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

        // Splitting a printed part is a last resort: only once plain removal
        // offers nowhere to go from this state.
        if (neighbours.empty())
            neighbours = findSplitNeighbours(current_node);

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
// `blockers`, when non-null, is filled with every assembled part that obstructs
// the upward lift.  The plain feasibility path stops at the first hit; only the
// split search needs the full set, and it needs it to know which printed part is
// worth cutting.
std::optional<gp_Pnt> Assembler::edge_feasible(
    std::shared_ptr<Part> part,
    const PartTransformMap& assembled,
    std::vector<std::shared_ptr<Part>>* blockers)
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

        // Built once per part and kept: rebuilding it per call churned shapes
        // through the adapter's pointer-keyed geometry cache, and is wasted work.
        auto& cached = lf_shape_cache_[part->getId()];
        if (!cached)
            cached = std::make_shared<TopoDS_Shape>(LocalFrameShapeM(*part->getShape()));
        auto part_lf_shape = cached;

        bool z_ok = true;
        for (int step = 1; step <= N_STEPS && (z_ok || blockers); ++step)
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
                    if (!blockers) break;   // fast path: one hit is enough

                    // Collecting: record every obstruction, not just the first.
                    if (std::find(blockers->begin(), blockers->end(), other) == blockers->end())
                        blockers->push_back(other);
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
    // Body-collision + seal checks were already done once for this part in
    // precomputeGraspCandidates(); only the (cheap) check against the parts
    // currently assembled at this DFS node needs to run here.
    std::vector<std::string> assembled_ids;
    assembled_ids.reserve(assembled.size());
    for (auto const& [other, _] : assembled)
    {
        if (other->getId() == part->getId()) continue;
        assembled_ids.push_back(other->getName() + "_" + std::to_string(other->getId()));
    }

    auto candidates_it = grasp_candidates_.find(part->getId());
    if (candidates_it == grasp_candidates_.end()) return std::nullopt;

    auto grasp_opt = VacuumGraspGenerator::select(
        part, candidates_it->second, *collision_adapter_, nozzle_shape_, assembled_ids);

    if (!grasp_opt)
    {
        // No seal available here — try the parallel-plate gripper instead.  A
        // gripper grasp is reported as a zero vacuum offset because the two are
        // parameterised differently; the tool actually used is recorded on the
        // part by generateGrasps().
        auto ppg_it = ppg_candidates_.find(part->getId());
        if (ppg_it != ppg_candidates_.end() &&
            PPGGraspGenerator::select(part, ppg_it->second, gripper_config_,
                                      *collision_adapter_, assembled_ids))
            return gp_Pnt(0, 0, 0);

        return std::nullopt;
    }
    const gp_Pnt grasp = *grasp_opt;

    return grasp;
}

namespace {

// Count solid bodies in a boolean result.  A horizontal plane can divide a part
// into more than two pieces (two towers cut below the fork gives one lower and
// two upper), which the brief rules out, so this is how that is detected.
int countSolids(const TopoDS_Shape& shape)
{
    int n = 0;
    for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) ++n;
    return n;
}

// A box spanning `reference` in X/Y with generous margin, covering [z_lo, z_hi].
TopoDS_Shape halfSpaceBox(const TopoDS_Shape& reference, double z_lo, double z_hi)
{
    Bnd_Box bb = ShapeBoundingBox(reference);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    const double m = 10.0;   // margin so the cut fully clears the part in X/Y
    return BRepPrimAPI_MakeBox(
        gp_Pnt(xmin - m, ymin - m, z_lo),
        gp_Pnt(xmax + m, ymax + m, z_hi)).Shape();
}

}  // namespace

// Split `shape` by the horizontal plane z = z_cut.  Returns false unless the cut
// yields exactly one solid on each side with meaningful volume on both — that
// enforces the two-piece rule and rejects planes that merely graze the part.
static bool splitAtZ(const TopoDS_Shape& shape, double z_cut,
                     TopoDS_Shape& lower, TopoDS_Shape& upper)
{
    Bnd_Box bb = ShapeBoundingBox(shape);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    const double pad = 10.0;
    if (z_cut <= zmin + 1e-6 || z_cut >= zmax - 1e-6) return false;  // nothing to divide

    try {
        upper = SubtractShapeBFromA(shape, halfSpaceBox(shape, zmin - pad, z_cut));
        lower = SubtractShapeBFromA(shape, halfSpaceBox(shape, z_cut, zmax + pad));
    } catch (const Standard_Failure&) {
        return false;
    }

    if (upper.IsNull() || lower.IsNull())              return false;
    if (countSolids(upper) != 1 || countSolids(lower) != 1) return false;

    const double vu = ShapeVolume(upper), vl = ShapeVolume(lower);
    if (vu < 1e-3 || vl < 1e-3) return false;

    return true;
}

// Make a Part out of a cut piece and register it with the scene and collision
// backend.  Pieces are added permanently rather than swapped in and out: the
// collision scene is global to the search, but what a node actually collides
// against is decided by its own assembled map, so extra geometry sitting in the
// scene is inert until a node lists it.
std::shared_ptr<Part> Assembler::registerSplitPiece(const TopoDS_Shape& shape,
                                                    const std::string&  name)
{
    auto piece = std::make_shared<Part>(
        std::make_shared<TopoDS_Shape>(shape), Part::INTERNAL, next_split_id_++, name);

    piece->set_mesh_asset(Tessellator::tessellate(shape, MESH_DEFLECTION_MM));

    const gp_Pnt c = ShapeCentroid(shape);
    gp_Trsf pose;
    pose.SetTranslation(gp_Vec(c.X() * 0.001, c.Y() * 0.001, c.Z() * 0.001));

    const std::string id = piece->getName() + "_" + std::to_string(piece->getId());
    scene_.add_object(id, piece->get_mesh_asset(),
                      std::make_shared<TopoDS_Shape>(LocalFrameShapeM(shape)),
                      SceneRole::Part, pose, true);

    // Pieces must be resolvable by ID downstream (getPartById), and printed
    // pieces need a slot on the print bed.  arrangeInternalParts() will give
    // this a real position later; the centroid is a placeholder until then.
    if (initial_assembly_)
        initial_assembly_->setUnassembledPart(piece, c);

    return piece;
}

void Assembler::unregisterSplitPiece(const std::shared_ptr<Part>& piece)
{
    if (!piece) return;
    scene_.remove_object(piece->getName() + "_" + std::to_string(piece->getId()));
    if (initial_assembly_) initial_assembly_->removeUnassembledPart(piece);
    lf_shape_cache_.erase(piece->getId());
}

// Advisory check on the first layer printed after the insertion.  Takes a thin
// slab of the upper piece just above the cut and asks how much of it lands on
// solid material (the lower piece or the part just placed).  Support is the
// model designer's responsibility, so this only warns.
void Assembler::warnIfUnsupported(const TopoDS_Shape& upper,
                                  const TopoDS_Shape& lower,
                                  const TopoDS_Shape& placed_part,
                                  double z_cut,
                                  const std::string& name) const
{
    constexpr double LAYER   = 0.4;   // mm, representative first-layer thickness
    constexpr double WARN_AT = 0.30;  // warn once 30% of the layer is unsupported

    try {
        TopoDS_Shape slab = ShapeIntersection(upper, halfSpaceBox(upper, z_cut, z_cut + LAYER));
        if (slab.IsNull()) return;
        const double slab_vol = ShapeVolume(slab);
        if (slab_vol < 1e-6) return;

        // Material directly beneath the cut, lifted by one layer so an
        // intersection with the slab measures the supported footprint.
        TopoDS_Shape below = makeCompound({
            ShapeIntersection(lower,       halfSpaceBox(lower,       z_cut - LAYER, z_cut)),
            ShapeIntersection(placed_part, halfSpaceBox(placed_part, z_cut - LAYER, z_cut))});
        below = TranslateShape(below, gp_Vec(0, 0, LAYER));

        TopoDS_Shape supported = ShapeIntersection(slab, below);
        const double supported_vol = supported.IsNull() ? 0.0 : ShapeVolume(supported);
        const double unsupported   = 1.0 - std::min(1.0, supported_vol / slab_vol);

        if (unsupported > WARN_AT)
            RCLCPP_WARN(logger(),
                        "Split of %s at z=%.2f leaves %.0f%% of the first printed layer "
                        "unsupported — check the model has support there",
                        name.c_str(), z_cut, unsupported * 100.0);
    } catch (const Standard_Failure&) {
        // Advisory only — a failed boolean here must not affect planning.
    }
}

// Successors reachable only by cutting a printed part in two.
//
// Called only when ordinary removal has no successor at all, so the search never
// cuts a part while a plain move exists.  For each external part that cannot be
// lifted, the printed parts blocking it are candidates for the cut; the plane is
// pinned to the top of the blocked part, which is the only height that works:
// lower and the printhead would strike the part when resuming the print, higher
// and the lower piece still encloses it so it could never have been inserted.
//
// Both phases are verified before either is committed, and emitted as one
// transition — the cut has no purpose except to free that part.
std::vector<std::shared_ptr<AssemblyNode>> Assembler::findSplitNeighbours(
    std::shared_ptr<AssemblyNode> node)
{
    std::vector<std::shared_ptr<AssemblyNode>> neighbours;
    if (splits_used_ >= max_splits_ || !collision_adapter_) return neighbours;

    // A search that dead-ends thousands of times would otherwise re-attempt this
    // at every one of them, and each attempt is several boolean operations.
    const int attempt_budget = std::max(10, 50 * max_splits_);
    if (split_attempts_ >= attempt_budget) return neighbours;

    node->assembly_->setPartTransforms();
    const auto& assembled = node->assembly_->getAssembledPartTransforms();

    // ---- candidate (blocked part, printed blocker) pairs ----
    struct Candidate {
        std::shared_ptr<Part> blocked;   // P
        std::shared_ptr<Part> printed;   // I
        double                z_cut;
    };
    std::vector<Candidate> candidates;

    for (auto const& [part, transform] : assembled)
    {
        if (part->getType() != Part::EXTERNAL) continue;

        std::vector<std::shared_ptr<Part>> blockers;
        if (edge_feasible(part, assembled, &blockers)) continue;   // not actually stuck

        for (auto const& b : blockers)
            if (b->getType() == Part::INTERNAL && b->getShape())
                candidates.push_back({part, b, ShapeHighestPoint(*part->getShape())});
    }

    if (candidates.empty()) return neighbours;

    // Cut as high as possible: the smaller the upper piece, the less has to be
    // reprinted after the insertion.
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b){ return a.z_cut > b.z_cut; });

    for (auto const& cand : candidates)
    {
        TopoDS_Shape lower, upper;
        if (!splitAtZ(*cand.printed->getShape(), cand.z_cut, lower, upper))
            continue;   // >2 pieces, or the plane divides nothing

        ++split_attempts_;
        if (split_attempts_ >= attempt_budget) {
            RCLCPP_INFO(logger(), "Split attempt budget (%d) reached — no more cutting",
                        attempt_budget);
            break;
        }

        auto piece_lower = registerSplitPiece(
            lower, cand.printed->getName() + "_lower");
        auto piece_upper = registerSplitPiece(
            upper, cand.printed->getName() + "_upper");
        collision_adapter_->sync(scene_);

        // Pieces are registered before they can be tested, so a rejected
        // candidate has to put the scene back — otherwise every speculative cut
        // the search ever makes stays in it, and both memory and sync cost grow
        // without bound over a long run.
        auto discard = [&] {
            unregisterSplitPiece(piece_lower);
            unregisterSplitPiece(piece_upper);
            collision_adapter_->sync(scene_);
        };

        // State after phase 1's cut, before anything is removed.
        PartTransformMap after_split;
        for (auto const& [p, t] : assembled)
            if (p->getId() != cand.printed->getId()) after_split[p] = t;
        after_split[piece_lower] = ShapeCentroid(lower);
        after_split[piece_upper] = ShapeCentroid(upper);

        // Phase 1: the upper piece must come off.
        if (!edge_feasible(piece_upper, after_split)) { discard(); continue; }

        // Phase 2: with it gone, the blocked part must come off.
        PartTransformMap after_upper = after_split;
        after_upper.erase(piece_upper);
        auto grasp = edge_feasible(cand.blocked, after_upper);
        if (!grasp) { discard(); continue; }

        warnIfUnsupported(upper, lower, *cand.blocked->getShape(),
                          cand.z_cut, cand.printed->getName());

        // ---- commit both phases as one transition ----
        auto next_assembly = std::make_shared<Assembly>();
        for (auto const& [p, t] : node->assembly_->getUnassembledPartTransforms())
            next_assembly->setUnassembledPart(p, t);
        for (auto const& [p, t] : after_upper)
            if (p->getId() != cand.blocked->getId()) next_assembly->setAssembledPart(p, t);

        next_assembly->setUnassembledPart(
            piece_upper, initial_assembly_->getUnassembledPartTransforms().count(piece_upper)
                             ? initial_assembly_->getUnassembledPartTransforms()[piece_upper]
                             : ShapeCentroid(upper));
        next_assembly->setUnassembledPart(
            cand.blocked, initial_assembly_->getUnassembledPartTransforms()[cand.blocked]);

        auto next = std::make_shared<AssemblyNode>();
        next->assembly_      = next_assembly;
        next->id_            = nodeIdGenerator(next_assembly->getAssembledPartIds());
        next->edge_part_     = cand.blocked;
        next->edge_grasp_    = *grasp;
        next->is_split_step_ = true;
        next->split_source_  = cand.printed;
        next->split_lower_   = piece_lower;
        next->split_upper_   = piece_upper;
        next->split_z_       = cand.z_cut;
        next->split_z_rel_   = cand.z_cut - ShapeLowestPoint(*cand.printed->getShape());

        ++splits_used_;
        RCLCPP_INFO(logger(),
                    "Split %s at z=%.2f mm to free %s (split %d of %d allowed)",
                    cand.printed->getName().c_str(), cand.z_cut,
                    cand.blocked->getName().c_str(), splits_used_, max_splits_);

        neighbours.push_back(next);
        break;   // one split per dead end; the search can split again later
    }

    return neighbours;
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
        {
            traceDfs("{\"ev\":\"reject\",\"from\":" + std::to_string(node->id_) +
                     ",\"part\":" + std::to_string(part->getId()) +
                     ",\"why\":\"printed part must come out last\"}");
            continue;
        }

        // When tracing, ask which parts obstruct the lift so a stuck part can be
        // explained rather than just reported as infeasible.
        std::vector<std::shared_ptr<Part>> blockers;
        auto grasp_result = edge_feasible(part, assembled,
                                          trace_dfs_ ? &blockers : nullptr);
        if (!grasp_result)
        {
            std::string why, by = "[";
            if (!blockers.empty()) {
                why = "obstructed";
                for (size_t i = 0; i < blockers.size(); ++i)
                    by += (i ? "," : "") + std::to_string(blockers[i]->getId());
            } else {
                why = "no grasp found";
            }
            by += "]";
            traceDfs("{\"ev\":\"reject\",\"from\":" + std::to_string(node->id_) +
                     ",\"part\":" + std::to_string(part->getId()) +
                     ",\"why\":\"" + why + "\",\"by\":" + by + "}");
            continue;
        }

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

        traceDfs("{\"ev\":\"edge\",\"from\":" + std::to_string(node->id_) +
                 ",\"to\":" + std::to_string(neighbour_node->id_) +
                 ",\"part\":" + std::to_string(part->getId()) +
                 ",\"parts\":" + idListJson(neighbour_assembly->getAssembledPartTransforms()) + "}");

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

    // Split pieces are created during the search, so their IDs must start above
    // every ID already in use.
    next_split_id_ = 0;
    for (auto const& [part, _] : target_assembly_->getAssembledPartTransforms())
        next_split_id_ = std::max(next_split_id_, part->getId() + 1);

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
    make_collision_adapter_ = [safety_margin_m]() {
        return std::make_unique<CoalAdapter>(safety_margin_m);
    };
    collision_adapter_ = make_collision_adapter_();
    collision_adapter_->sync(scene_);

    RCLCPP_INFO(logger(), "SceneModel built with %zu objects (Coal, safety margin %.4f mm)",
                scene_.present_object_ids().size(),
                safety_margin_m * 1000.0);

    precomputeGraspCandidates();
}

// Compute geometry-only grasp candidates for every external part once, in
// parallel — see VacuumGraspGenerator::precompute for what's cached and why
// it's safe to defer the assembled-parts check to select() at DFS time.
// Each worker gets its own CollisionAdapter instance (via
// make_collision_adapter_) so no thread touches the shared collision_adapter_.
void Assembler::precomputeGraspCandidates()
{
    grasp_candidates_.clear();
    ppg_candidates_.clear();
    if (!generate_grasps_ || !nozzle_shape_ || !make_collision_adapter_) return;

    struct Job {
        std::shared_ptr<Part>         part;
        gp_Pnt                        world_pos_mm;
        std::shared_ptr<TopoDS_Shape> local_frame_shape;
    };
    std::vector<Job> jobs;
    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms())
        if (part->getType() == Part::EXTERNAL && part->getShape())
            jobs.push_back({part, transform, nullptr});

    if (jobs.empty()) return;

    // Build and tessellate every shape the worker threads below will touch
    // — the nozzle and each part's local-frame collision copy — serially,
    // before any thread starts.  BRepMesh_IncrementalMesh mutates the
    // shape's internal triangulation cache; nozzle_shape_ in particular is
    // the exact same object every worker reads, and duplicate/instanced
    // parts (e.g. repeated screws) can share underlying OCCT geometry too,
    // so triangulating lazily on first concurrent use would race. Once meshed
    // here, every worker only ever reads already-built triangulation data.
    const double deflection_m = MESH_DEFLECTION_MM * 0.001;
    {
        BRepMesh_IncrementalMesh nozzle_mesher(*nozzle_shape_, deflection_m);
        nozzle_mesher.Perform();
    }
    for (auto& job : jobs) {
        job.local_frame_shape = std::make_shared<TopoDS_Shape>(
            LocalFrameShapeM(*job.part->getShape()));
        BRepMesh_IncrementalMesh mesher(*job.local_frame_shape, deflection_m);
        mesher.Perform();
    }

    RCLCPP_INFO(logger(), "Precomputing grasp candidates for %zu external part(s)...",
                jobs.size());

    std::mutex results_mutex;
    std::atomic<size_t> next_index{0};

    const unsigned n_threads = std::max(1u, std::min(
        std::thread::hardware_concurrency(),
        static_cast<unsigned>(jobs.size())));

    auto worker = [&]() {
        for (;;) {
            const size_t i = next_index.fetch_add(1);
            if (i >= jobs.size()) return;

            const auto& job = jobs[i];
            auto adapter = make_collision_adapter_();
            PartGraspCandidates candidates = VacuumGraspGenerator::precompute(
                job.part, job.world_pos_mm, job.local_frame_shape, *adapter, nozzle_shape_);

            // Parallel-plate candidates for the same part.  Pure geometry — no
            // adapter involved — so it rides along on the same worker.
            PartPPGCandidates ppg =
                PPGGraspGenerator::precompute(job.part, gripper_config_);

            std::lock_guard<std::mutex> lock(results_mutex);
            grasp_candidates_[job.part->getId()] = std::move(candidates);
            ppg_candidates_[job.part->getId()]   = std::move(ppg);
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(n_threads);
    for (unsigned t = 0; t < n_threads; ++t)
        pool.emplace_back(worker);
    for (auto& th : pool)
        th.join();

    RCLCPP_INFO(logger(), "Grasp candidate precompute done (%u threads)", n_threads);

    // Per-part summary.  These counts are geometry-only: a part with candidates
    // here can still end up ungraspable once select() tests them against the
    // parts actually assembled at a given DFS node.  A part with zero candidates
    // has no viable grasp on its own geometry at all, independent of the assembly.
    for (auto const& job : jobs)
    {
        const auto& faces = grasp_candidates_[job.part->getId()].faces;
        size_t total = 0;
        for (auto const& f : faces) total += f.size();
        RCLCPP_INFO(logger(),
                    "  %-40s vacuum: %zu candidate(s) on %zu face(s), gripper: %zu",
                    job.part->getName().c_str(), total, faces.size(),
                    ppg_candidates_[job.part->getId()].candidates.size());
    }
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

        // A part picked with the gripper needs channels through the cradle wall
        // for its fingers.  The grasp is stored relative to the part centroid, so
        // it re-registers onto the part wherever generateNegatives() has just put
        // it.  Slots are cut with clearance and swept up past the jig top, so the
        // jaws can descend into them from above.
        if (part->getGraspTool() == Part::GraspTool::PPG)
        {
            const PPGGrasp& g = part->getPPGGrasp();
            const gp_Pnt c = ShapeCentroid(*part->getShape());

            PPGCandidate cand;
            cand.angle_rad = g.rotation_;
            cand.width_mm  = g.width_;
            cand.jaw_z_mm  = c.Z() + g.jaw_z_;
            cand.centre    = gp_Pnt(c.X() + g.position_.X(),
                                    c.Y() + g.position_.Y(),
                                    c.Z() + g.position_.Z());

            GripperConfig slot_cfg = gripper_config_;
            slot_cfg.jaw_width_mm     += 2.0 * slot_cfg.slot_clearance_mm;
            slot_cfg.jaw_thickness_mm += 2.0 * slot_cfg.slot_clearance_mm;

            const double sweep_top = c.Z() + JIG_HEIGHT + 20.0;

            std::vector<TopoDS_Shape> slots;
            for (int side : {+1, -1})
                slots.push_back(PPGGraspGenerator::jawSolid(cand, slot_cfg, side, sweep_top));
            cradle_gen.setFingerSlots(slots);

            RCLCPP_INFO(logger(), "Jig for %s slotted for gripper at %.1f deg, opening %.2f mm",
                        part->getName().c_str(), g.rotation_ * 180.0 / M_PI, g.width_);
        }

        RCLCPP_INFO(logger(), "Cradle for %s with %.2f mm hull clearance",
                    part->getName().c_str(), cradle_scaling_distance_);

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
// Update every scene object's pose from the target assembly's current transforms
// and re-sync the collision backend.
//
// The scene is populated once, in generateInitialAssembly().  But
// alignAssemblyPathToInitialAssembly() later translates the whole assembly onto
// the printed base part, which leaves the collision objects behind at their
// pre-alignment poses.  Any collision query made after that point then tests the
// nozzle against geometry hundreds of mm away and passes vacuously — the OCCT-only
// seal check still fires, so the failure looks like a plausible result rather than
// an obviously broken one.  Callers that query collisions must refresh first.
//
// Only poses need updating: the cached local-frame shapes are built about each
// shape's own bbox centroid, so a pure translation leaves them unchanged.
void Assembler::refreshCollisionScene()
{
    if (!collision_adapter_ || !target_assembly_) return;

    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms())
    {
        const std::string id = part->getName() + "_" + std::to_string(part->getId());
        if (!scene_.has_object(id)) continue;

        gp_Trsf pose;
        pose.SetTranslation(gp_Vec(transform.X() * 0.001,
                                   transform.Y() * 0.001,
                                   transform.Z() * 0.001));
        scene_.set_pose(id, pose);
    }

    collision_adapter_->sync(scene_);
}

// Build the list of (part, assembled-part-IDs) pairs to grasp-check.
//
// A part is only ever grasped at its own step in the sequence, against whatever
// is already in place at that moment.  Checking it against the *finished*
// assembly instead asks a question the robot never faces: a part placed early
// can be completely enclosed by the time the build is done, so a perfectly good
// grasp is reported as impossible.  MACH_1's motor is exactly that case — it goes
// on first, onto the bare chassis, but ends up surrounded by the clamp, two bolts
// and two gears.
//
// Falls back to the full assembly when there is no path (jig/grasp-only mode),
// where there is no sequence to take a step from.
std::vector<std::pair<std::shared_ptr<Part>, std::vector<std::string>>>
Assembler::graspEvaluationContexts() const
{
    std::vector<std::pair<std::shared_ptr<Part>, std::vector<std::string>>> contexts;

    if (!assembly_path_.empty())
    {
        for (auto const& node : assembly_path_)
        {
            if (!node || !node->edge_part_) continue;
            auto part = node->edge_part_;
            if (part->getType() != Part::EXTERNAL) continue;

            // The node's assembled set includes the edge part itself; the nozzle
            // only has to clear everything that was already there.
            std::vector<std::string> ids;
            for (auto const& [other, _] : node->assembly_->getAssembledPartTransforms())
            {
                if (other->getId() == part->getId()) continue;
                ids.push_back(other->getName() + "_" + std::to_string(other->getId()));
            }
            contexts.emplace_back(part, std::move(ids));
        }
        return contexts;
    }

    const auto& all_parts = target_assembly_->getAssembledPartTransforms();
    for (auto const& [part, transform] : all_parts)
    {
        if (part->getType() != Part::EXTERNAL) continue;

        std::vector<std::string> ids;
        ids.reserve(all_parts.size() - 1);
        for (auto const& [other, _] : all_parts)
        {
            if (other->getId() == part->getId()) continue;
            ids.push_back(other->getName() + "_" + std::to_string(other->getId()));
        }
        contexts.emplace_back(part, std::move(ids));
    }
    return contexts;
}

// Install a hand-edited grasp on a part, filling in whatever the override does
// not carry (jaw angle and height) from the planner's own best candidate, and
// reporting whether the result still passes the usual checks.
void Assembler::applyGraspOverride(const std::shared_ptr<Part>& part,
                                   const GraspOverride& ov,
                                   const std::vector<std::string>& assembled_ids)
{
    if (ov.tool == "gripper")
    {
        PPGGrasp g;
        g.position_ = gp_Vec(ov.dx_mm, ov.dy_mm, ov.dz_mm);
        g.valid_    = true;

        // Angle and jaw height are not part of the edit, so take them from the
        // best candidate the planner found; failing that, a neutral pose.
        auto it = ppg_candidates_.find(part->getId());
        if (it != ppg_candidates_.end() && !it->second.candidates.empty())
        {
            const auto& best = it->second.candidates.front();
            const gp_Pnt c = ShapeCentroid(*part->getShape());
            g.rotation_ = best.angle_rad;
            g.jaw_z_    = best.jaw_z_mm - c.Z();
            g.width_    = best.width_mm;
        }
        else
        {
            g.rotation_ = 0.0;
            g.jaw_z_    = ov.dz_mm - 0.5 * gripper_config_.jaw_height_mm;
            g.width_    = gripper_config_.max_opening_mm * 0.5;
        }
        if (ov.width_mm > 0.0) g.width_ = ov.width_mm;
        if (ov.has_angle)      g.rotation_ = ov.angle_rad;

        part->setPPGGrasp(g);
        part->setGraspTool(Part::GraspTool::PPG);
        RCLCPP_INFO(logger(),
                    "generateGrasps: %s uses a manual gripper grasp "
                    "(%.2f, %.2f, %.2f) mm, opening %.2f mm, angle %.1f deg",
                    part->getName().c_str(), ov.dx_mm, ov.dy_mm, ov.dz_mm, g.width_,
                    g.rotation_ * 180.0 / M_PI);
    }
    else
    {
        part->setVacuumGrasp(gp_Pnt(ov.dx_mm, ov.dy_mm, ov.dz_mm));
        part->setGraspTool(Part::GraspTool::VACUUM);
        RCLCPP_INFO(logger(),
                    "generateGrasps: %s uses a manual vacuum grasp (%.2f, %.2f, %.2f) mm",
                    part->getName().c_str(), ov.dx_mm, ov.dy_mm, ov.dz_mm);
    }
    (void)assembled_ids;
}

void Assembler::generateGrasps()
{
    if (!collision_adapter_ || !nozzle_shape_) return;

    // Ensure shapes are at assembly positions for consistent OCCT queries.
    target_assembly_->setPartTransforms();
    refreshCollisionScene();

    grasp_attempts_.clear();

    for (auto const& [part, assembled_ids] : graspEvaluationContexts())
    {
        // (cancellation hook — currently a no-op; add std::atomic<bool>* flag if needed)

        // A hand-edited grasp replaces synthesis outright.  It is still checked,
        // so a warning goes out if the user has moved it somewhere the planner
        // would have rejected — but their choice is what gets emitted.
        auto ov = grasp_overrides_.find(part->getId());
        if (ov != grasp_overrides_.end())
        {
            applyGraspOverride(part, ov->second, assembled_ids);
            continue;
        }

        auto it = grasp_candidates_.find(part->getId());
        if (it == grasp_candidates_.end())
        {
            RCLCPP_WARN(logger(), "generateGrasps: no precomputed candidates for %s",
                        part->getName().c_str());
            continue;
        }

        // Same call the DFS made at this part's node, with the same cached
        // candidates and the same parts in place — so this reproduces the grasp
        // the sequence was validated with, rather than searching for a new one.
        // debug_out captures the full picture for the visualiser at no extra cost.
        auto grasp = VacuumGraspGenerator::select(
            part, it->second, *collision_adapter_, nozzle_shape_,
            assembled_ids, &grasp_attempts_);

        if (grasp)
        {
            part->setVacuumGrasp(*grasp);
            part->setGraspTool(Part::GraspTool::VACUUM);
            continue;
        }

        // Vacuum cannot seal here — fall back to the parallel-plate gripper.
        auto ppg_it = ppg_candidates_.find(part->getId());
        std::optional<PPGGrasp> ppg;
        if (ppg_it != ppg_candidates_.end())
            ppg = PPGGraspGenerator::select(part, ppg_it->second, gripper_config_,
                                            *collision_adapter_, assembled_ids);

        if (ppg)
        {
            part->setPPGGrasp(*ppg);
            part->setGraspTool(Part::GraspTool::PPG);
        }
        else
        {
            part->setGraspTool(Part::GraspTool::NONE);
            RCLCPP_WARN(logger(),
                        "generateGrasps: no grasp found for %s with either tool "
                        "(against %zu part(s) in place)",
                        part->getName().c_str(), assembled_ids.size());
        }
    }
}

// The attempts recorded by generateGrasps().  This deliberately does NOT
// re-run the search: doing so used to produce a second, independent answer that
// could disagree with the grasp the machine was given.  Positions are kept in
// world mm and are translated along with the assembly by
// alignAssemblyPathToInitialAssembly().
std::vector<GraspAttempt> Assembler::debugGrasps()
{
    return grasp_attempts_;
}