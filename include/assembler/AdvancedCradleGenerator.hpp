#ifndef ADVANCED_CRADLE_GENERATOR_HPP
#define ADVANCED_CRADLE_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class AdvancedCradleGenerator
{
public:
    AdvancedCradleGenerator(std::string name, TopoDS_Shape shape, float scaling_distance = 0.2f)
        : shape_(shape), name_(name), scaling_distance_(scaling_distance) {}

    float createJig(float bay_size, int bay_index);

private:
    TopoDS_Shape shape_;
    std::string name_;
    float scaling_distance_;
};

#endif // ADVANCED_CRADLE_GENERATOR_HPP
