#ifndef SPARSE_JIG_GENERATOR_HPP
#define SPARSE_JIG_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class SparseJigGenerator
{
public:
    SparseJigGenerator(std::string name, TopoDS_Shape shape, float sampling_resolution = 5.0f)
        : shape_(shape), name_(name), sampling_resolution_(sampling_resolution) {}

    // Build and return the raw jig geometry (no STL write, no JIG_CENTER_Z translation).
    TopoDS_Shape buildJigShape(float bay_size);

    // Build jig, translate to JIG_CENTER_Z, write STL to output_dir, return part-relative z offset.
    float createJig(float bay_size, int bay_index, const std::string& output_dir);

private:
    TopoDS_Shape shape_;
    std::string name_;
    float sampling_resolution_;  // spatial sampling step R (mm)
};

#endif // SPARSE_JIG_GENERATOR_HPP
