#ifndef ARMS_CONFIG_HPP
#define ARMS_CONFIG_HPP

#include "assembler/MeshFunctions.hpp"

const extern double PARTS_BED_HEIGHT;

const extern std::vector<std::vector<gp_Pnt>> PARTS_BAY_POSITIONS;

const extern std::vector<int> BAY_SIZES;

const extern double PRINT_BED_CENTER[2];

const extern double PRINT_BED_BOTTOM_LEFT[2];

const extern double PRINT_BED_TOP_RIGHT[2];

const extern double PRINT_MIN_SPACING;

const extern double PRINT_BED_HEIGHT;

const extern double JIG_HEIGHT;

const extern double JIG_CENTER_Z;

// BRepMesh chord tolerance used for the canonical part tessellation (mm).
// The CoalAdapter safety margin is MESH_DEFLECTION_MM * 0.5 * 0.001 m.
const extern double MESH_DEFLECTION_MM;

#endif