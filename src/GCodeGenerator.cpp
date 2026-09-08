#include "assembler/GCodeGenerator.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/Part.hpp"
#include "assembler/Config.hpp"
#include "assembler/Logger.hpp"

void GCodeGenerator::generate(std::shared_ptr<Assembly> initial_assembly,
                              std::shared_ptr<Assembly> target_assembly,
                              std::shared_ptr<Part> base_part,
                              std::vector<size_t> part_addition_order,
                              const std::vector<std::vector<std::string>>& printer_gcode_segments,
                              const std::map<size_t, size_t>& print_segment_after_part,
                              const std::string& output_dir)
{
    std::vector<std::string> gcode;

    auto emit_segment = [&](size_t index, bool resuming) {
        if (index >= printer_gcode_segments.size()) return;
        if (resuming)
        {
            // The vacuum tool is still fitted from the insertion — swap back to
            // the extruder before laying down the next layer.
            gcode.push_back(";RESUME PRINT COMMAND");
            toolChangeExtruder(gcode);
            moveToSafeHeight(gcode);
        }
        else
        {
            gcode.push_back(";PRINT COMMAND");
        }
        for (const std::string& print_line : printer_gcode_segments[index])
            gcode.push_back(print_line);
    };

    emit_segment(0, false);

    //Iterate through the part addition order, find the part in the initial and target assemblies, generate the correct actions
    for (size_t part_id : part_addition_order)
    {
        //Ignore base part - shouldn't this just be the first part in the list?
        if (part_id == base_part->getId() && base_part->getType() == Part::INTERNAL)
            continue;

        std::shared_ptr<Part> part = initial_assembly->getPartById(part_id);

        Part::PART_TYPE part_type = part->getType();

        if (part_type == Part::EXTERNAL)
        {
            gp_Pnt initial_transform = initial_assembly->getUnassembledPartTransforms()[part];

            gp_Vec pick_position = SumPoints(initial_transform, part->getGraspOffset());

            RCLCPP_INFO(logger(), "Pick transform: %f %f %f grasp %f %f %f", initial_transform.X(), initial_transform.Y(), initial_transform.Z(), part->getVacuumGrasp().X(), part->getVacuumGrasp().Y(), part->getVacuumGrasp().Z());

            gp_Vec place_position = SumPoints(target_assembly->getAssembledPartTransforms()[part], part->getGraspOffset());

            gcode.push_back(";PLACE EXTERNAL PART COMMAND " + part->getName());

            moveToSafeHeight(gcode);

            homeY(gcode);

            homeX(gcode);

            const bool use_gripper = (part->getGraspTool() == Part::GraspTool::PPG);
            if (use_gripper)
            {
                const PPGGrasp& g = part->getPPGGrasp();
                gcode.push_back(";  using parallel plate gripper");
                toolChangeGripper(gcode);
                // Align the jaws to the grasp axis and open them clear of the part
                // before descending into the jig's finger slots.
                gcode.push_back("GRIPPER_ROTATE A=" + std::to_string(g.rotation_ * 180.0 / M_PI));
                gcode.push_back("GRIPPER_OPEN W=" + std::to_string(g.width_ + 6.0));
            }
            else
            {
                toolChangeVacuum(gcode);
            }

            moveToSafeHeight(gcode);

            moveToPosition(gcode, pick_position.X(), pick_position.Y(), 2000);

            moveToHeight(gcode, pick_position.Z() - 2, 2000);    //Including offset for vacuum nozzle     //TODO need to check this

            if (use_gripper)
                gcode.push_back("GRIPPER_CLOSE W=" + std::to_string(part->getPPGGrasp().width_));
            else
                vacuumOn(gcode);

            wait(gcode, 500);

            moveToSafeHeight(gcode, 2000);

            moveToPosition(gcode, place_position.X(), place_position.Y(), 2000);

            moveToHeight(gcode, place_position.Z() - 2, 2000);    //Including offset for vacuum nozzle

            if (use_gripper)
                gcode.push_back("GRIPPER_OPEN W=" + std::to_string(part->getPPGGrasp().width_ + 6.0));
            else
                vacuumOff(gcode);

            wait(gcode, 500);

            moveToSafeHeight(gcode, 2000);

            // If the print was interrupted to drop this part in, carry on now.
            auto resume = print_segment_after_part.find(part_id);
            if (resume != print_segment_after_part.end())
                emit_segment(resume->second, true);
        }

        else if (part_type == Part::SCREW)
        {            
            gp_Pnt initial_transform = initial_assembly->getUnassembledPartTransforms()[part];

            gp_Vec place_position = SumPoints(target_assembly->getAssembledPartTransforms()[part], gp_Pnt(0, 0, 2.0));    //TODO need to calibrate fixing offset

            gcode.push_back(";PLACE FIXING COMMAND");

            moveToSafeHeight(gcode);

            homeY(gcode);

            homeX(gcode);

            toolChangeDriver(gcode);

            moveToSafeHeight(gcode, 2000);

            moveToPosition(gcode, place_position.X(), place_position.Y(), 2000);

            moveToHeight(gcode, place_position.Z(), 2000);

            insertScrew(gcode);

            wait(gcode, 6000);

            moveToSafeHeight(gcode, 2000);
        }      
    }

    toolDropoff(gcode);

    std::ofstream fout(output_dir + "gcode.gcode");

    for (std::string s : gcode)
    {
        fout << s << '\n';
    }

    fout.close();
}

void GCodeGenerator::toolDropoff(std::vector<std::string> &gcode)
{
    gcode.push_back("TOOL_DROPOFF");
}

void GCodeGenerator::toolChangeExtruder(std::vector<std::string> &gcode)
{
    gcode.push_back("TOOL_PICKUP T=0 ;Extruder");
}

void GCodeGenerator::toolChangeVacuum(std::vector<std::string> &gcode)
{
    gcode.push_back("TOOL_PICKUP T=2 ;Vacuum gripper");
}

void GCodeGenerator::toolChangeGripper(std::vector<std::string> &gcode)
{
    gcode.push_back("TOOL_PICKUP T=3 ;Parallel plate gripper");
}

void GCodeGenerator::toolChangeDriver(std::vector<std::string> &gcode)
{
    gcode.push_back("TOOL_PICKUP T=4 ;Screwdriver");
}

void GCodeGenerator::moveToPosition(std::vector<std::string> &gcode, float x, float y, int feed)
{
    if (feed == -1)
        gcode.push_back("G0 X" + std::to_string(x) + " Y" + std::to_string(y));

    else
        gcode.push_back("G0 X" + std::to_string(x) + " Y" + std::to_string(y) + " F" + std::to_string(feed));
}

void GCodeGenerator::moveToHeight(std::vector<std::string> &gcode, float z, int feed)
{
    gcode.push_back("G0 Z" + std::to_string(z) + " F" + std::to_string(feed));
}

void GCodeGenerator::moveToSafeHeight(std::vector<std::string> &gcode, int feed)
{
    moveToHeight(gcode, 100, feed);
}

void GCodeGenerator::insertScrew(std::vector<std::string> &gcode)
{
    gcode.push_back("INSERT_SCREW SCREW=long");
}

void GCodeGenerator::vacuumOn(std::vector<std::string> &gcode)
{
    gcode.push_back("VACUUM_ON");
}

void GCodeGenerator::vacuumOff(std::vector<std::string> &gcode)
{
    gcode.push_back("VACUUM_OFF");
}

void GCodeGenerator::wait(std::vector<std::string> &gcode, float duration)
{
    gcode.push_back("G4 P" + std::to_string(duration));
}

void GCodeGenerator::homeX(std::vector<std::string> &gcode)
{
    gcode.push_back("G28 X");
}

void GCodeGenerator::homeY(std::vector<std::string> &gcode)
{
    gcode.push_back("G28 Y");
}