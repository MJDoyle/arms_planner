"""Scene document I/O for .arms files produced by arms-plan."""

from __future__ import annotations

import json
import struct
import zipfile
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass
class Pose:
    x: float
    y: float
    z: float

    @classmethod
    def _from_dict(cls, d: dict) -> "Pose":
        return cls(x=float(d["x"]), y=float(d["y"]), z=float(d["z"]))


@dataclass
class PartRecord:
    part_id: int
    name: str
    type: str           # "internal" | "external" | "screw"
    mesh_file: str
    assembled_pose: Pose
    bay_pose: Pose


@dataclass
class GraspRecord:
    x_mm: float
    y_mm: float
    z_mm: float
    status: str         # "accepted" | "body_collision" | "assembly_collision" | "seal_failed"
    part_id: Optional[int] = None  # absent in .arms files written before this field existed
    # True on the one accepted candidate per part that the planner actually
    # picked (closest to the centre of mass).  False on the other accepted
    # candidates, which were viable but not selected.  Absent in older files.
    chosen: bool = False


@dataclass
class StageRecord:
    step: int
    assembled_part_ids: list[int]
    unassembled_part_ids: list[int]
    edge_part_id: Optional[int] = None
    edge_grasp: Optional[Pose] = None


@dataclass
class JigRecord:
    stl_file: str


@dataclass
class BackgroundRecord:
    """A static environment mesh baked into the .arms file by arms-plan.

    The GLB's vertices are already in world-space metres — arms-plan applies
    the pose from background_models.txt at write time — so unlike parts these
    need no per-object placement, only the viewer's scene-centring offset.
    """
    file: str
    r: float = 0.5      # base colour, linear 0..1
    g: float = 0.5
    b: float = 0.5

    @property
    def color_rgb255(self) -> tuple[int, int, int]:
        """Base colour as the 0-255 int triple viser expects."""
        return tuple(  # type: ignore[return-value]
            max(0, min(255, int(round(c * 255))))
            for c in (self.r, self.g, self.b)
        )


# ---------------------------------------------------------------------------
# GLB parser (reads the subset written by GlbWriter.cpp)
# ---------------------------------------------------------------------------

def _parse_glb(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    """Return (vertices float32 Nx3, triangles uint32 Mx3) from a GLB buffer."""
    magic, _version, _length = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67:
        raise ValueError("Not a valid glTF binary file")

    json_len, json_type = struct.unpack_from("<II", data, 12)
    if json_type != 0x4E4F534A:
        raise ValueError("Expected JSON chunk first")
    gltf = json.loads(data[20 : 20 + json_len])

    bin_start = 20 + json_len
    _bin_len, bin_type = struct.unpack_from("<II", data, bin_start)
    if bin_type != 0x004E4942:
        raise ValueError("Expected BIN chunk second")
    bin_data = data[bin_start + 8 :]

    # Accessor 0: float32 VEC3 positions
    acc_pos = gltf["accessors"][0]
    bv_pos = gltf["bufferViews"][acc_pos["bufferView"]]
    pos_off = bv_pos.get("byteOffset", 0)
    n_verts = acc_pos["count"]
    verts = np.frombuffer(bin_data, dtype=np.float32, count=n_verts * 3, offset=pos_off)
    verts = verts.reshape(n_verts, 3).copy()

    # Accessor 1: uint32 SCALAR indices
    acc_idx = gltf["accessors"][1]
    bv_idx = gltf["bufferViews"][acc_idx["bufferView"]]
    idx_off = bv_idx.get("byteOffset", 0)
    n_idx = acc_idx["count"]
    idx = np.frombuffer(bin_data, dtype=np.uint32, count=n_idx, offset=idx_off)
    tris = idx.reshape(-1, 3).copy()

    return verts, tris


# ---------------------------------------------------------------------------
# SceneDocument
# ---------------------------------------------------------------------------

@dataclass
class SceneDocument:
    """Deserialized content of a .arms file."""

    version: str
    source_cad: str
    generated: str
    pipeline_commit: str
    parts: list[PartRecord]
    stages: list[StageRecord]
    grasps: list[GraspRecord]
    jigs: list[JigRecord]
    gcode_file: str
    command_file: str

    # Static environment meshes.  Defaulted so .arms files written before
    # background support existed still load.
    background: list[BackgroundRecord] = field(default_factory=list)

    # GLB of the vacuum nozzle the planner used, in local-frame metres with its
    # contact face at z = -0.01 m.  Empty for files written before it was baked in.
    nozzle_file: str = ""

    # Set when loaded from a file; None if constructed programmatically.
    arms_path: Optional[str] = field(default=None, compare=False)

    # ---------------------------------------------------------------------------

    @classmethod
    def load(cls, arms_path: str) -> "SceneDocument":
        """Load a SceneDocument from a .arms zip file."""
        with zipfile.ZipFile(arms_path, "r") as zf:
            with zf.open("manifest.json") as f:
                data = json.load(f)

        parts = [
            PartRecord(
                part_id=p["part_id"],
                name=p["name"],
                type=p["type"],
                mesh_file=p["mesh_file"],
                assembled_pose=Pose._from_dict(p["assembled_pose"]),
                bay_pose=Pose._from_dict(p["bay_pose"]),
            )
            for p in data.get("parts", [])
        ]

        stages = [
            StageRecord(
                step=s["step"],
                assembled_part_ids=list(s["assembled_part_ids"]),
                unassembled_part_ids=list(s["unassembled_part_ids"]),
                edge_part_id=s.get("edge_part_id"),
                edge_grasp=Pose._from_dict(s["edge_grasp"]) if s.get("edge_grasp") else None,
            )
            for s in data.get("stages", [])
        ]

        grasps = [
            GraspRecord(
                x_mm=float(g["x_mm"]),
                y_mm=float(g["y_mm"]),
                z_mm=float(g["z_mm"]),
                status=g["status"],
                part_id=g.get("part_id"),
                chosen=bool(g.get("chosen", False)),
            )
            for g in data.get("grasps", [])
        ]

        jigs = [JigRecord(stl_file=j["stl_file"]) for j in data.get("jigs", [])]

        background = [
            BackgroundRecord(
                file=b["file"],
                r=float(b.get("r", 0.5)),
                g=float(b.get("g", 0.5)),
                b=float(b.get("b", 0.5)),
            )
            for b in data.get("background", [])
        ]

        return cls(
            version=data["version"],
            source_cad=data["source_cad"],
            generated=data["generated"],
            pipeline_commit=data["pipeline_commit"],
            parts=parts,
            stages=stages,
            grasps=grasps,
            jigs=jigs,
            gcode_file=data.get("gcode_file", ""),
            command_file=data.get("command_file", ""),
            background=background,
            nozzle_file=data.get("nozzle_file", ""),
            arms_path=arms_path,
        )

    def mesh_vertices_triangles(
        self, part: PartRecord
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (vertices float32 Nx3 in metres, triangles uint32 Mx3) for a part."""
        glb_bytes = self.read_entry(part.mesh_file)
        return _parse_glb(glb_bytes)

    def background_vertices_triangles(
        self, bg: BackgroundRecord
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (vertices float32 Nx3, triangles uint32 Mx3) for a background mesh.

        Vertices are already world-space metres, not local-frame like parts.
        """
        return _parse_glb(self.read_entry(bg.file))

    def nozzle_vertices_triangles(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (vertices float32 Nx3 in metres, triangles uint32 Mx3) for the nozzle.

        Vertices are local-frame: the nozzle centroid is at the origin, so placing
        this at a GraspRecord's (x_mm, y_mm, z_mm) puts the tool exactly where the
        planner tested it.  Raises if the file carries no nozzle.
        """
        if not self.nozzle_file:
            raise RuntimeError("this .arms file contains no nozzle mesh")
        return _parse_glb(self.read_entry(self.nozzle_file))

    def read_entry(self, entry_name: str) -> bytes:
        """Read a raw entry from the .arms zip."""
        if self.arms_path is None:
            raise RuntimeError("SceneDocument was not loaded from a .arms file")
        with zipfile.ZipFile(self.arms_path, "r") as zf:
            return zf.read(entry_name)

    def part_by_id(self, part_id: int) -> Optional[PartRecord]:
        for p in self.parts:
            if p.part_id == part_id:
                return p
        return None
