#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"

#include "assembler/Config.hpp"

#include "assembler/Logger.hpp"

#include <cmath>
#include <stdexcept>

/*  Input: ptr to another part

    Output: collision between parts yes/no

    First check the bounding boxes of the two parts to see if they intersect. If they do, use an OCC function
    to check the closest distance between them
*/
double Part::collisionVolume(std::shared_ptr<Part> otherPart)
{
    Bnd_Box this_bb = ShapeBoundingBox(*shape_);
    Bnd_Box other_bb = ShapeBoundingBox(*(otherPart->getShape()));

    if (this_bb.IsOut(other_bb))
        return 0.0;

    TopoDS_Shape intersection = ShapeIntersection(*shape_, *(otherPart->getShape()));

    if (intersection.IsNull())
        return 0.0;

    return ShapeVolume(intersection);
}

bool Part::collide(std::shared_ptr<Part> otherPart)
{
    float collisionThreshold = 0.01;                                
    Bnd_Box this_bb = ShapeBoundingBox(*shape_);
    Bnd_Box other_bb = ShapeBoundingBox(*(otherPart->getShape()));

    if (this_bb.IsOut(other_bb))
        return false;

    BRepExtrema_DistShapeShape distCalc(*shape_, *(otherPart->getShape()));
    
    if (!distCalc.IsDone()) 
    {
        RCLCPP_ERROR(logger(), "Collision can't be calculated");

        return true;  // Distance can't be computed - assume collision
    }

    if (distCalc.Value() < collisionThreshold)
        return true;
    
    else
        return false;
}





gp_Pnt Part::generateBayPosition(std::vector<std::vector<bool>>& occupancy)
{
    double substrate_depth = 6;   
    bay_size_index_ = 0;
    bay_index_ = -1;                                 //The bay index for a given size
    Standard_Real x_size = ShapeAxisSize(*shape_, 0);
    Standard_Real y_size = ShapeAxisSize(*shape_, 1);

    //Select correct bay size depending on size of part
    if (x_size > 38 || y_size > 38)
    {
        bay_size_index_ = 1;
    }

    //Find a free, appropriately sized bay
    for (int i = 0; i < occupancy[bay_size_index_].size(); i ++)
    {
        if (occupancy[bay_size_index_][i] == false)
        {
            bay_index_ = i;
            break;
        }
    }

    if (bay_index_ == -1)
        throw std::runtime_error("Part::generateBayPosition: no free bay for part '" + name_ + "'");

    occupancy[bay_size_index_][bay_index_] = true;

    RCLCPP_INFO(logger(), "Bay index %d, bay size index %d", bay_index_, bay_size_index_);

    has_bay_assigned_ = true;
    bay_position_ = gp_Pnt(PARTS_BAY_POSITIONS[bay_size_index_][bay_index_].X(),
                            PARTS_BAY_POSITIONS[bay_size_index_][bay_index_].Y(),
                            ShapeAxisSize(*shape_, 2) / 2 - 4);
    return bay_position_;

}

