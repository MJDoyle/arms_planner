#ifndef RAFT_GENERATOR_HPP
#define RAFT_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

#include <memory>

class Part;
class Assembly;

class RaftGenerator
{
    static void generate(std::shared_ptr<Part> base_part, std::shared_ptr<Assembly> target_assembly);
};

#endif