#include "assembler/GlbWriter.hpp"

#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

// ---------------------------------------------------------------------------
// Binary glTF 2.0 writer for a single triangle mesh.
//
// Format reference: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html
//   Header  : 12 bytes  (magic, version, total length)
//   Chunk 0 : JSON      (chunkLength, chunkType=0x4E4F534A, data padded w/ spaces)
//   Chunk 1 : BIN       (chunkLength, chunkType=0x004E4942, data padded w/ zeros)
// ---------------------------------------------------------------------------

static void write_u32(std::vector<uint8_t>& buf, uint32_t v)
{
    buf.push_back(v & 0xFF);
    buf.push_back((v >> 8) & 0xFF);
    buf.push_back((v >> 16) & 0xFF);
    buf.push_back((v >> 24) & 0xFF);
}

static void write_f32(std::vector<uint8_t>& buf, float v)
{
    uint32_t bits;
    std::memcpy(&bits, &v, 4);
    write_u32(buf, bits);
}

std::vector<uint8_t> mesh_to_glb(const MeshAsset& mesh)
{
    const size_t n_verts   = mesh.vertices.size();
    const size_t n_tris    = mesh.triangles.size();
    const size_t n_indices = n_tris * 3;

    // ----- Build binary buffer: float32 positions then uint32 indices -----
    std::vector<uint8_t> bin;
    bin.reserve(n_verts * 12 + n_indices * 4);

    float xmin =  std::numeric_limits<float>::max();
    float ymin =  std::numeric_limits<float>::max();
    float zmin =  std::numeric_limits<float>::max();
    float xmax = -std::numeric_limits<float>::max();
    float ymax = -std::numeric_limits<float>::max();
    float zmax = -std::numeric_limits<float>::max();

    for (auto const& v : mesh.vertices)
    {
        float x = static_cast<float>(v[0]);
        float y = static_cast<float>(v[1]);
        float z = static_cast<float>(v[2]);
        write_f32(bin, x);  write_f32(bin, y);  write_f32(bin, z);
        xmin = std::min(xmin, x);  ymin = std::min(ymin, y);  zmin = std::min(zmin, z);
        xmax = std::max(xmax, x);  ymax = std::max(ymax, y);  zmax = std::max(zmax, z);
    }

    const size_t index_byte_offset = bin.size();

    for (auto const& tri : mesh.triangles)
    {
        write_u32(bin, static_cast<uint32_t>(tri[0]));
        write_u32(bin, static_cast<uint32_t>(tri[1]));
        write_u32(bin, static_cast<uint32_t>(tri[2]));
    }

    // Pad binary chunk to multiple of 4
    while (bin.size() % 4 != 0) bin.push_back(0);

    // ----- Build JSON chunk -----
    auto fmt_f = [](float v) -> std::string {
        std::ostringstream ss;
        ss << v;
        return ss.str();
    };

    const size_t vert_byte_len  = n_verts * 12;
    const size_t idx_byte_len   = n_indices * 4;
    const size_t total_bin_len  = vert_byte_len + idx_byte_len;

    std::ostringstream json;
    json << R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],)"
         << R"("meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1,"mode":4}]}],)"
         << R"("accessors":[)"
         <<   R"({"bufferView":0,"componentType":5126,"count":)" << n_verts
         <<     R"(,"type":"VEC3","min":[)" << fmt_f(xmin) << "," << fmt_f(ymin) << "," << fmt_f(zmin)
         <<     R"(],"max":[)" << fmt_f(xmax) << "," << fmt_f(ymax) << "," << fmt_f(zmax) << "]},"
         <<   R"({"bufferView":1,"componentType":5125,"count":)" << n_indices << R"(,"type":"SCALAR"})"
         << R"(],"bufferViews":[)"
         <<   R"({"buffer":0,"byteOffset":0,"byteLength":)" << vert_byte_len << R"(,"target":34962},)"
         <<   R"({"buffer":0,"byteOffset":)" << index_byte_offset << R"(,"byteLength":)" << idx_byte_len << R"(,"target":34963})"
         << R"(],"buffers":[{"byteLength":)" << total_bin_len << "}]}";

    std::string json_str = json.str();
    // Pad JSON to multiple of 4 with spaces (spec-required)
    while (json_str.size() % 4 != 0) json_str += ' ';

    // ----- Assemble GLB -----
    const uint32_t total_len =
        12                                         // header
        + 8 + static_cast<uint32_t>(json_str.size())   // JSON chunk header + data
        + 8 + static_cast<uint32_t>(bin.size());        // BIN  chunk header + data

    std::vector<uint8_t> glb;
    glb.reserve(total_len);

    // Header
    write_u32(glb, 0x46546C67u);  // magic 'glTF'
    write_u32(glb, 2u);           // version
    write_u32(glb, total_len);

    // JSON chunk
    write_u32(glb, static_cast<uint32_t>(json_str.size()));
    write_u32(glb, 0x4E4F534Au);  // 'JSON'
    glb.insert(glb.end(), json_str.begin(), json_str.end());

    // BIN chunk
    write_u32(glb, static_cast<uint32_t>(bin.size()));
    write_u32(glb, 0x004E4942u);  // 'BIN\0'
    glb.insert(glb.end(), bin.begin(), bin.end());

    return glb;
}

// ---------------------------------------------------------------------------
// GLB reader — the exact inverse of mesh_to_glb().
// ---------------------------------------------------------------------------

static uint32_t read_u32_le(const uint8_t* p)
{
    uint32_t v;
    std::memcpy(&v, p, 4);
    return v;
}

MeshAsset glb_to_mesh(const std::vector<uint8_t>& glb)
{
    if (glb.size() < 12)
        throw std::runtime_error("glb_to_mesh: file too small");

    if (read_u32_le(glb.data()) != 0x46546C67u)
        throw std::runtime_error("glb_to_mesh: not a GLB file (bad magic)");

    // JSON chunk at offset 12
    const uint32_t json_len  = read_u32_le(glb.data() + 12);
    const uint32_t json_type = read_u32_le(glb.data() + 16);
    if (json_type != 0x4E4F534Au)
        throw std::runtime_error("glb_to_mesh: expected JSON chunk type");

    const std::string json_str(reinterpret_cast<const char*>(glb.data() + 20), json_len);

    // Extract the two "count" values from the accessors array.
    // First occurrence → n_verts (POSITION accessor), second → n_indices (INDEX accessor).
    auto next_count = [&](size_t from) -> std::pair<uint32_t, size_t>
    {
        size_t pos = json_str.find("\"count\":", from);
        if (pos == std::string::npos)
            throw std::runtime_error("glb_to_mesh: 'count' not found in JSON chunk");
        size_t s = pos + 8;
        while (s < json_str.size() && json_str[s] == ' ') ++s;
        size_t e = s;
        while (e < json_str.size() && std::isdigit(static_cast<unsigned char>(json_str[e]))) ++e;
        if (e == s) throw std::runtime_error("glb_to_mesh: expected number after \"count\":");
        return {static_cast<uint32_t>(std::stoul(json_str.substr(s, e - s))), e};
    };

    auto [n_verts,   pos1] = next_count(0);
    auto [n_indices, pos2] = next_count(pos1);
    const uint32_t n_tris  = n_indices / 3;

    // BIN chunk follows the JSON chunk (both are already 4-byte padded by the writer)
    const size_t bin_hdr = 20 + json_len;
    if (bin_hdr + 8 > glb.size())
        throw std::runtime_error("glb_to_mesh: BIN chunk out of bounds");
    if (read_u32_le(glb.data() + bin_hdr + 4) != 0x004E4942u)
        throw std::runtime_error("glb_to_mesh: expected BIN chunk type");

    const uint8_t* bin = glb.data() + bin_hdr + 8;

    MeshAsset mesh;

    // Positions: n_verts * 3 * float32 = n_verts * 12 bytes at offset 0
    mesh.vertices.resize(n_verts);
    for (uint32_t i = 0; i < n_verts; ++i)
    {
        float x, y, z;
        std::memcpy(&x, bin + i * 12,     4);
        std::memcpy(&y, bin + i * 12 + 4, 4);
        std::memcpy(&z, bin + i * 12 + 8, 4);
        mesh.vertices[i] = {static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
    }

    // Indices: n_tris * 3 * uint32 = n_tris * 12 bytes at offset n_verts*12
    // n_verts*12 is always 4-aligned (12 = 3*4), so no extra padding needed.
    const size_t idx_base = n_verts * 12;
    mesh.triangles.resize(n_tris);
    for (uint32_t i = 0; i < n_tris; ++i)
    {
        uint32_t a, b, c;
        std::memcpy(&a, bin + idx_base + i * 12,     4);
        std::memcpy(&b, bin + idx_base + i * 12 + 4, 4);
        std::memcpy(&c, bin + idx_base + i * 12 + 8, 4);
        mesh.triangles[i] = {static_cast<int>(a), static_cast<int>(b), static_cast<int>(c)};
    }

    return mesh;
}

size_t write_glb(const MeshAsset& mesh, const std::string& path)
{
    auto data = mesh_to_glb(mesh);
    std::ofstream f(path, std::ios::binary);
    if (!f) return 0;
    f.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    return data.size();
}
