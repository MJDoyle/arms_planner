#include "assembler/RaftGenerator.hpp"

#include "assembler/Assembly.hpp"

#include "assembler/Part.hpp"


void RaftGenerator::generate(std::shared_ptr<Part> base_part, std::shared_ptr<Assembly> target_assembly)
{
    //Get position from this
    //target_assembly->getAssembledPartTransforms[base_part]

    //Then get lowest point

    //Get xy bounding box

    //Make a raft up to just below the lowest point, including ribs - need to test
}