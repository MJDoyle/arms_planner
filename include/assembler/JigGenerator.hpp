#ifndef JIG_GENERATOR_HPP
#define JIG_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class JigGenerator
{
public:
    JigGenerator(std::string name, TopoDS_Shape shape, float scaling_distance = 0.2f)
        : shape_(shape), name_(name), scaling_distance_(scaling_distance) {}

    // Build and return the raw jig geometry (no STL write, no JIG_CENTER_Z translation).
    // The shape is in world coordinates relative to the part's current centroid position.
    TopoDS_Shape buildJigShape(float bay_size);

    // Build jig, translate to JIG_CENTER_Z, write STL to output_dir, return part-relative z offset.
    float createJig(float bay_size, int bay_index, const std::string& output_dir);

private:
    TopoDS_Shape shape_;
    std::string name_;
    float scaling_distance_;
};

#endif // JIG_GENERATOR_HPP
