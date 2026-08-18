#ifndef TEST_CRADLE_GENERATOR_HPP
#define TEST_CRADLE_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class TestCradleGenerator
{
public:
    TestCradleGenerator(std::string name, TopoDS_Shape shape, float scaling_distance = 0.2f)
        : shape_(shape), name_(name), scaling_distance_(scaling_distance) {}

    float createJig(float bay_size, int bay_index);

private:
    TopoDS_Shape shape_;
    std::string name_;
    float scaling_distance_;
};

#endif // TEST_CRADLE_GENERATOR_HPP
