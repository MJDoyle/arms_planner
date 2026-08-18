#pragma once

#include <array>
#include <memory>
#include <vector>

// Local-frame triangulated mesh in metres, shared across all consumers (Foxglove,
// collision adapter, future USD).  Produced once per part by Tessellator::tessellate.
struct MeshAsset {
    // Vertex positions in the part's local frame (bbox centroid at origin), metres.
    std::vector<std::array<double, 3>> vertices;
    // Triangle vertex indices into `vertices`.
    std::vector<std::array<int, 3>> triangles;
    // The BRepMesh chord tolerance used to produce this mesh (mm).
    // The safety margin in CollisionAdapter is derived from this value.
    double linear_deflection_mm{0.0};
};
