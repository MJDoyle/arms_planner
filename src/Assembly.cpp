#include "assembler/Assembly.hpp"

#include "assembler/Part.hpp"

#include "assembler/Logger.hpp"

#include <iostream>
#include <stdexcept>

// Default log sink: write to stderr.  main() or the ROS node wrapper replaces this.
std::function<void(int, const std::string&)> g_arms_log_sink =
    [](int level, const std::string& msg) {
        const char* prefix = "INFO";
        if      (level >= arms_log::FATAL) prefix = "FATAL";
        else if (level >= arms_log::ERROR) prefix = "ERROR";
        else if (level >= arms_log::WARN)  prefix = "WARN";
        else if (level <= arms_log::DEBUG) prefix = "DEBUG";
        std::fprintf(stderr, "[arms/%s] %s\n", prefix, msg.c_str());
    };

bool PartIdOrder::operator()(const std::shared_ptr<Part>& a,
                              const std::shared_ptr<Part>& b) const
{
    return a->getId() < b->getId();
}

#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <StlAPI_Writer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

void Assembly::setPartTransforms()
{
    for (auto const& [part, transform] : unassembled_part_transforms_)
    {
        part->setCentroidPosition(transform);
    }

    for (auto const& [part, transform] : assembled_part_transforms_)
    {
        part->setCentroidPosition(transform);
    }
}

std::vector<size_t> Assembly::getAssembledPartIds()
{
    std::vector<size_t> part_ids;

    for (auto const& [part, transform] : assembled_part_transforms_)
    {
        part_ids.push_back(part->getId());
    }

    return part_ids;
}

std::shared_ptr<Part> Assembly::getPartById(size_t id)
{
    for (auto const& [part, transform] : unassembled_part_transforms_)
    {
        if (part->getId() == id)
            return part;
    }

    for (auto const& [part, transform] : assembled_part_transforms_)
    {
        if (part->getId() == id)
            return part;
    }

    throw std::runtime_error("Assembly::getPartById: no part with id " + std::to_string(id));
}

int Assembly::getNumInternalParts()
{
    int num_internal_parts = 0;

    for (auto const& [part, transform] : unassembled_part_transforms_)
    {
        if (part->getType() == Part::INTERNAL)
            num_internal_parts ++;
    }

    for (auto const& [part, transform] : assembled_part_transforms_)
    {
        if (part->getType() == Part::INTERNAL)
            num_internal_parts ++;
    }

    return num_internal_parts;
}

// void Assembly::placeOnPoint(gp_Pnt point)
// {
//     //Find x,y center and lowest z point
//     std::cout << std::endl << "Placing on point: " << point.X() << " " << point.Y() << " " << point.Z() << std::endl;

//     //Find the bounding box of the entire assembly
//     double xmin = std::numeric_limits<double>::max();
//     double ymin = std::numeric_limits<double>::max();
//     double zmin = std::numeric_limits<double>::max();
//     double xmax = std::numeric_limits<double>::lowest();
//     double ymax = std::numeric_limits<double>::lowest();
//     double zmax = std::numeric_limits<double>::lowest();

//     for (std::shared_ptr<Part> part : parts_)
//     {
//         Bnd_Box bbox = ShapeBoundingBox(*(part->getShape()));

//         Standard_Real bb_xmin, bb_ymin, bb_zmin, bb_xmax, bb_ymax, bb_zmax;

//         bbox.Get(bb_xmin, bb_ymin, bb_zmin, bb_xmax, bb_ymax, bb_zmax);

//         xmin = std::min(xmin, bb_xmin);
//         ymin = std::min(ymin, bb_ymin);
//         zmin = std::min(zmin, bb_zmin);
//         xmax = std::max(xmax, bb_xmax);
//         ymax = std::max(ymax, bb_ymax);
//         zmax = std::max(zmax, bb_zmax);
//     }

//     double lowest_z = zmin;

//     gp_Pnt center((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0);

//     //Move al parts so that x,y lies on point.x and point.y, and lowest z point lies on zero (or adjust for bed heighgt)

//     gp_Vec translation(point, gp_Pnt(center.X(), center.Y(), lowest_z));

//     for (std::shared_ptr<Part> part : parts_)
//     {
//         part->translate(translation);
//     }
// }
