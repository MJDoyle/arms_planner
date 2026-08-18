#pragma once

#include "assembler/MeshAsset.hpp"

#include <TopoDS_Shape.hxx>
#include <memory>

namespace Tessellator {

// Tessellate `shape` with the given BRepMesh chord tolerance (mm).
// Vertices are stored in local frame (bbox centroid at origin), converted to metres.
// This is the single tessellation point — call once per part at load time and share
// the result via shared_ptr across all consumers (collision, visualisation, USD).
std::shared_ptr<MeshAsset> tessellate(const TopoDS_Shape& shape, double deflection_mm);

}  // namespace Tessellator
