#ifndef GCODEGENERATOR_HPP
#define GCODEGENERATOR_HPP

#include <vector>
#include <map>
#include <memory>

class Assembly;

class Part;

class GCodeGenerator
{
    public:

    // `printer_gcode_segments` is the sliced print, divided wherever a part has to
    // be inserted mid-build.  Segment 0 runs first; `print_segment_after_part`
    // maps a part ID to the segment that resumes once that part is in place.
    // With nothing split there is a single segment and an empty map, which
    // reproduces the original print-everything-then-assemble order.
    static void generate(std::shared_ptr<Assembly> initial_assembly,
                         std::shared_ptr<Assembly> target_assembly,
                         std::shared_ptr<Part> base_part,
                         std::vector<size_t> part_addition_order,
                         const std::vector<std::vector<std::string>>& printer_gcode_segments,
                         const std::map<size_t, size_t>& print_segment_after_part,
                         const std::string& output_dir);

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
