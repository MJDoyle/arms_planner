#ifndef SPARSE_CRADLE_GENERATOR_HPP
#define SPARSE_CRADLE_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class SparseCradleGenerator
{
public:
    SparseCradleGenerator(std::string name, TopoDS_Shape shape, float scaling_distance = 0.2f)
        : shape_(shape), name_(name), scaling_distance_(scaling_distance) {}

    float createJig(float bay_size, int bay_index);

private:
    TopoDS_Shape shape_;
    std::string name_;
    float scaling_distance_;
};

#endif // SPARSE_CRADLE_GENERATOR_HPP
