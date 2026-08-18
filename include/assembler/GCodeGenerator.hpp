#ifndef GCODEGENERATOR_HPP
#define GCODEGENERATOR_HPP

#include <vector>
#include <memory>

class Assembly;

class Part;

class GCodeGenerator
{
    public:

    static void generate(std::shared_ptr<Assembly> initial_assembly, std::shared_ptr<Assembly> target_assembly, std::shared_ptr<Part> base_part, std::vector<size_t> part_addition_order, std::vector<std::string> printer_gcode, const std::string& output_dir);

    static void toolDropoff(std::vector<std::string> &gcode);

    static void toolChangeExtruder(std::vector<std::string> &gcode);

    static void toolChangeVacuum(std::vector<std::string> &gcode);

    static void toolChangeGripper(std::vector<std::string> &gcode);

    static void toolChangeDriver(std::vector<std::string> &gcode);

    static void moveToPosition(std::vector<std::string> &gcode, float x, float y, int feed = -1);

    static void moveToHeight(std::vector<std::string> &gcode, float z, int feed = 3000);

    static void moveToSafeHeight(std::vector<std::string> &gcode, int feed = 3000);

    static void insertScrew(std::vector<std::string> &gcode);

    static void vacuumOn(std::vector<std::string> &gcode);

    static void vacuumOff(std::vector<std::string> &gcode);

    static void wait(std::vector<std::string> &gcode, float duration);

    static void homeX(std::vector<std::string> &gcode);

    static void homeY(std::vector<std::string> &gcode);
};

#endif
