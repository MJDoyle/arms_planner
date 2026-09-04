#ifndef ASSEMBLY_HPP
#define ASSEMBLY_HPP

#include "assembler/MeshFunctions.hpp"

#include <memory>
#include <vector>
#include <set>
#include <map>

class Part;

// Comparator that orders Part shared_ptrs by their numeric ID.
// This makes std::map iteration order deterministic across runs (ASLR changes
// pointer addresses; part IDs are assigned sequentially in ModelLoader).
struct PartIdOrder {
    bool operator()(const std::shared_ptr<Part>& a,
                    const std::shared_ptr<Part>& b) const;
};

// Canonical map type used throughout the planner.  All callers that previously
// wrote std::map<shared_ptr<Part>, gp_Pnt> should use this alias instead.
using PartTransformMap = std::map<std::shared_ptr<Part>, gp_Pnt, PartIdOrder>;

class Assembly {

public:
    explicit Assembly() {}

    void setAssembledPart(std::shared_ptr<Part> part, gp_Pnt position) { assembled_part_transforms_[part] = position; }

    void setUnassembledPart(std::shared_ptr<Part> part, gp_Pnt position) { unassembled_part_transforms_[part] = position; }

    // Used when a printed part is superseded by the pieces it was split into.
    void removeUnassembledPart(std::shared_ptr<Part> part) { unassembled_part_transforms_.erase(part); }
    void removeAssembledPart(std::shared_ptr<Part> part)   { assembled_part_transforms_.erase(part); }

    PartTransformMap getAssembledPartTransforms() { return assembled_part_transforms_; }

    PartTransformMap getUnassembledPartTransforms() { return unassembled_part_transforms_; }

    void setPartTransforms();

    //void setParts(PartTransformMap parts) { parts_ = parts; }

    std::shared_ptr<Part> getPartById(size_t id);   //TODO replace with a dictionary of ID to part in assembler?

    std::vector<size_t> getAssembledPartIds();

    int getNumInternalParts();

    // void placeOnPoint(gp_Pnt point);

    // void alignToPart(std::shared_ptr<Part> part);

    // void saveAsSTL(std::string filename);


private:

    PartTransformMap unassembled_part_transforms_;

    PartTransformMap assembled_part_transforms_;
};

struct AssemblyNode {

    std::shared_ptr<Assembly> assembly_;

    size_t id_;

    std::set<size_t> parent_ids_;

    // Part placed to reach this node from its parent (null for the root node).
    std::shared_ptr<Part> edge_part_;
    // Grasp for edge_part_ (local frame, relative to centroid, mm).
    // Zero for parts that don't need a grasp (internal, screws).
    gp_Pnt edge_grasp_;

    // ---- Split step (optional second phase) ----
    // Set when this node was reached by splitting a printed part to free
    // edge_part_.  The transition then has two phases, in this order:
    //   1. split split_source_ at z = split_z_, remove split_upper_
    //   2. remove edge_part_
    // They are recorded together because the split exists only to enable that
    // removal — taking one without the other would cut a part for nothing.
    // In assembly (forward) order this reads: print split_lower_, place
    // edge_part_, then print split_upper_ on top.
    bool                  is_split_step_ = false;
    std::shared_ptr<Part> split_source_;   // the printed part that was cut
    std::shared_ptr<Part> split_lower_;    // piece that stays in place
    std::shared_ptr<Part> split_upper_;    // piece removed in phase 1
    double                split_z_ = 0.0;  // cut height, world mm
    // Cut height measured from the printed part's own underside.  Frame-independent,
    // so it survives the later alignment translation and maps straight onto the
    // slicer's Z (parts are printed with their base on the bed at Z=0).
    double                split_z_rel_ = 0.0;

    bool operator <(const AssemblyNode& rhs) const
    {
        return id_ < rhs.id_;
    }

    bool operator ==(const AssemblyNode& rhs) const
    {
        return id_ == rhs.id_;
    }
};

#endif  // ASSEMBLY_HPP