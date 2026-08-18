#pragma once
#include "assembler/MeshAsset.hpp"
#include <string>

// Write a binary glTF 2.0 (.glb) file for the given mesh.
// Vertex positions are written as float32 POSITION in metres.
// Triangle indices are written as uint32.
// Returns the byte size of the written file, or 0 on failure.
size_t write_glb(const MeshAsset& mesh, const std::string& path);

// Serialise mesh to a GLB byte buffer (same content as write_glb but in memory).
std::vector<uint8_t> mesh_to_glb(const MeshAsset& mesh);

// Deserialise a GLB byte buffer produced by mesh_to_glb() back into a MeshAsset.
// Only understands the specific format written by mesh_to_glb (float32 POSITION,
// uint32 indices, single mesh, no extras).  Throws std::runtime_error on format errors.
MeshAsset glb_to_mesh(const std::vector<uint8_t>& glb);
