"""ARMS Planner — integrated planning + 3D viewer.

The viewer serves via viser (WebSocket on localhost:8080) and is wrapped in
a native OS window via pywebview so no external browser is needed.

Entry point:
    launch_app(step_dir)   — full integrated UI (planning + viewing)

Coordinate convention
---------------------
MeshAsset vertices are in metres (Tessellator multiplies OCCT mm coords × 0.001).
All pose and grasp values in the manifest are in millimetres.
The viewer works entirely in metres; all mm values are multiplied by MM = 1e-3.
Jig STLs (written by OCCT) are also in mm and scaled the same way.
All scene positions are expressed relative to a session-fixed origin (normally
the centre of the machine's background geometry) so that loading a plan never
shifts the world under the camera.  See the startup-camera block below.
"""

from __future__ import annotations

import glob
import json
import os
import re
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import viser

from arms_planner_core import SceneDocument, PartRecord

from .search_graph import SearchGraph, SearchGraphServer, build_figure

MM = 1e-3  # mm → metres

# Default directory for .step input files — arms_planner/input_models/
# Falls back to cwd if the directory doesn't exist (e.g. installed outside the repo).
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
_DEFAULT_STEP_DIR = _REPO_ROOT / "input_models"

# Same file arms-plan reads when baking background meshes into a .arms.  The
# viewer loads it directly so the machine is visible before any plan exists;
# the copy embedded in a .arms is only used as a fallback (see _ensure_background).
_BACKGROUND_MODELS_FILE = _REPO_ROOT / "config" / "background_models.txt"

# End-effector models, baked into each .arms so the viewer draws the real
# hardware rather than stand-in primitives.
_TOOL_MODEL_DIR = _REPO_ROOT / "models" / "tools"


def _plan_common_args(jig_clearance_mm: float) -> list[str]:
    """Planner flags shared by a fresh plan and a regeneration from edits."""
    args = _tool_model_args()
    cfg = _REPO_ROOT / "config"
    if cfg.exists():
        args += ["--slicer-config-dir", str(cfg) + "/"]
    return args + ["--jig-clearance", f"{jig_clearance_mm:g}", "--trace-dfs"]


def _tool_model_args() -> list[str]:
    """Planner flags for whichever tool models are present."""
    args: list[str] = []
    for flag, stem in (("--vacuum-model", "vacuum"), ("--gripper-model", "gripper")):
        path = _TOOL_MODEL_DIR / f"{stem}.step"
        if path.exists():
            args += [flag, str(path)]
    return args


# ---------------------------------------------------------------------------
# Startup camera
#
# The scene is always re-centred so the assembly sits at world origin (see
# _ViewerState.origin), so these are in metres relative to the assembly, not
# to the print bed — they stay sensible across different models.
#
# Camera state in viser is per-client, not a scene property, so this is applied
# to each ClientHandle when it connects — and only then.  Loading a plan never
# moves the camera, so a view you have dragged to is yours until you change it.
#
# The scene origin is likewise fixed for the session (see _scene_origin), so
# geometry never shifts underneath a stationary camera either.
# ---------------------------------------------------------------------------

# Used when background models are configured: the scene is anchored on the
# machine, which is far larger than any one assembly, so the view is wide.
_CAMERA_POSITION_OVERVIEW: tuple[float, float, float] = (0, -0.3, 0.3)
_CAMERA_LOOK_AT_OVERVIEW:  tuple[float, float, float] = (0.00,  0.00, 0.00)

# Used when there is no machine geometry to anchor to, so the scene is anchored
# on the assembly instead and wants closer framing.
_CAMERA_POSITION: tuple[float, float, float] = (0.18, -0.26, 0.18)
_CAMERA_LOOK_AT:  tuple[float, float, float] = (0.00,  0.00, -0.02)


# ---------------------------------------------------------------------------
# Colour palettes — viser requires (R, G, B) integers 0-255
# ---------------------------------------------------------------------------

_OPAQUE: dict[str, tuple[int, int, int]] = {
    "external": (64,  153, 242),
    "internal": (217, 140,  51),
    "screw":    (153, 217, 102),
    "unknown":  (178, 178, 178),
}

# Solid colours for unassembled (in-bay) parts — deliberately distinct from
# both _OPAQUE (assembled) and _JIG_COLOUR (orange) so bay parts, assembled
# parts, and jigs all stay visually distinguishable now that none of them
# use transparency.
_UNASSEMBLED: dict[str, tuple[int, int, int]] = {
    "external": (140, 195, 242),
    "internal": (150, 160, 217),
    "screw":    (200, 242, 160),
    "unknown":  (220, 220, 220),
}

_GRASP_COLOURS: dict[str, tuple[int, int, int]] = {
    "accepted":           ( 51, 217,  77),
    "body_collision":     (230,  51,  51),
    "assembly_collision": (242, 153,  26),
    "seal_failed":        (179,  51, 230),
}

# The one grasp per part the planner actually selected.  Deliberately distinct
# from the green "accepted" candidates it was chosen from, and drawn larger so
# it reads as the answer rather than one more candidate.
_GRASP_CHOSEN_COLOUR = (26, 140, 255)

# The chosen grasp is drawn as the actual vacuum nozzle baked into the .arms,
# in a steel tone that reads as a tool against the parts and jigs.  Files with
# no nozzle mesh (written before it was included) fall back to a plain marker.
_NOZZLE_COLOUR = (122, 138, 158)
_GRASP_RADIUS        = 3e-3
_GRASP_CHOSEN_RADIUS = 5e-3

# The gripper jaws, in the same steel tone as the nozzle so both read as tooling.
_JAW_COLOUR = (122, 138, 158)


def _box_mesh(lo: tuple[float, float, float],
              hi: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned box as (vertices, triangles)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ], dtype=np.float32)
    t = np.array([
        [0, 2, 1], [0, 3, 2],   # bottom
        [4, 5, 6], [4, 6, 7],   # top
        [0, 1, 5], [0, 5, 4],
        [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6],
        [3, 0, 4], [3, 4, 7],
    ], dtype=np.uint32)
    return v, t


_JIG_COLOUR = (204, 128, 26)

# Part types that are only shown once assembled.  Fixings live in the feeder
# rather than a jig bay, so drawing them at a bay position before they are
# placed is misleading — they simply appear as they go in.
_HIDE_WHEN_UNASSEMBLED: set[str] = {"screw"}

# Part types hidden while unassembled unless the user asks for them.  Printed
# parts are laid out on the print bed rather than in a jig, so before they are
# assembled they show where they will be *printed* — useful occasionally, but
# noise most of the time, hence off by default.
_OPTIONAL_WHEN_UNASSEMBLED: set[str] = {"internal"}


# ---------------------------------------------------------------------------
# STL parser (for jig files on disk)
# ---------------------------------------------------------------------------

def _parse_stl(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    """Return (vertices_mm float32 Nx3, triangles uint32 Mx3)."""
    if b'\x00' not in data[:256]:
        try:
            verts, tris = _parse_ascii_stl(data)
            if len(verts) > 0:
                return verts, tris
        except Exception:
            pass
    return _parse_binary_stl(data)


def _parse_binary_stl(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    if len(data) < 84:
        raise ValueError("STL too small")
    n_tris = struct.unpack_from('<I', data, 80)[0]
    dtype = np.dtype([
        ('normal', np.float32, (3,)),
        ('v0',     np.float32, (3,)),
        ('v1',     np.float32, (3,)),
        ('v2',     np.float32, (3,)),
        ('attr',   np.uint16),
    ])
    tri_data = np.frombuffer(data, dtype=dtype, count=n_tris, offset=84)
    verts = np.empty((n_tris * 3, 3), dtype=np.float32)
    verts[0::3] = tri_data['v0']
    verts[1::3] = tri_data['v1']
    verts[2::3] = tri_data['v2']
    return verts, np.arange(n_tris * 3, dtype=np.uint32).reshape(-1, 3)


def _parse_ascii_stl(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    verts, tris, cur = [], [], []
    for line in data.decode('ascii', errors='replace').splitlines():
        tok = line.split()
        if tok and tok[0] == 'vertex' and len(tok) == 4:
            cur.append([float(tok[1]), float(tok[2]), float(tok[3])])
        elif tok and tok[0] == 'endloop':
            if len(cur) == 3:
                base = len(verts)
                verts.extend(cur)
                tris.append([base, base + 1, base + 2])
            cur = []
    return np.array(verts, dtype=np.float32), np.array(tris, dtype=np.uint32)


# ---------------------------------------------------------------------------
# Background models loaded straight from config/background_models.txt
#
# These mirror parse_background_entry() / load_stl_world_space() in main.cpp so
# the viewer can show the machine without a plan.  Keep the two in step: the
# line format is "path,r,g,b,x,y,z,qx,qy,qz,qw" with translation in mm, and the
# world transform is  p_world_m = 0.001 * (R(q) @ p_mm + t_mm).
# ---------------------------------------------------------------------------

def _parse_background_models_file(path: Path) -> list[tuple]:
    """Return [(stl_path, (r,g,b) 0-255, t_mm (3,), quat (qx,qy,qz,qw)), ...]."""
    entries: list[tuple] = []
    try:
        text = path.read_text()
    except OSError:
        return entries

    for line_no, line in enumerate(text.splitlines(), start=1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(",")
        if len(fields) < 11:
            print(f"[viewer] WARNING: {path.name}:{line_no}: expected 11 fields, "
                  f"got {len(fields)} — skipping", file=sys.stderr)
            continue
        try:
            nums = [float(f) for f in fields[-10:]]
        except ValueError:
            print(f"[viewer] WARNING: {path.name}:{line_no}: non-numeric field — skipping",
                  file=sys.stderr)
            continue
        # Everything before the last 10 fields is the path (tolerates commas in it),
        # matching the C++ parser.
        stl_path = ",".join(fields[:-10]).strip()
        r, g, b = nums[0:3]
        colour = tuple(max(0, min(255, int(round(c * 255)))) for c in (r, g, b))
        entries.append((stl_path, colour,
                        np.array(nums[3:6], dtype=np.float64),
                        np.array(nums[6:10], dtype=np.float64)))
    return entries


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrix from (qx, qy, qz, qw)."""
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def _load_background_models() -> list[tuple]:
    """Load every entry in the config file as (vertices_world_m, triangles, colour)."""
    meshes: list[tuple] = []
    for stl_path, colour, t_mm, quat in _parse_background_models_file(_BACKGROUND_MODELS_FILE):
        try:
            with open(stl_path, "rb") as f:
                verts_mm, tris = _parse_stl(f.read())
        except Exception as e:
            print(f"[viewer] WARNING: background STL failed {stl_path}: {e}", file=sys.stderr)
            continue
        if verts_mm.size == 0 or tris.size == 0:
            print(f"[viewer] WARNING: background STL has no geometry: {stl_path}",
                  file=sys.stderr)
            continue
        world_m = ((_quat_to_matrix(quat) @ verts_mm.astype(np.float64).T).T + t_mm) * MM
        meshes.append((world_m.astype(np.float32), tris.astype(np.uint32), colour))
    return meshes


# ---------------------------------------------------------------------------
# Scene state — holds viser handles for a loaded .arms document
# ---------------------------------------------------------------------------

class _ViewerState:
    def __init__(self, doc: SceneDocument, server: viser.ViserServer,
                 origin: np.ndarray | None = None):
        self.doc      = doc
        self.server   = server
        self.n_stages = len(doc.stages)
        self._fixed_origin = origin

        self.part_handles:           dict[int, object] = {}
        self.part_checkboxes:        dict[int, object] = {}
        # Three disjoint groups, each independently toggleable:
        #   failed     — candidates rejected by a collision or seal check
        #   candidate  — passed every check, but not the one picked
        #   chosen     — the single grasp per part the planner selected
        self.grasp_handles_failed:    list             = []
        self.grasp_handles_candidate: list             = []
        self.grasp_handles_chosen:    list             = []
        # Gripper jaw plates, kept with the chosen grasps so one toggle covers both.
        self.jaw_records:             list             = []
        self.grasp_records:          list              = []
        self.jig_handles:            list              = []
        self.background_handles:     list              = []

        self.current_stage = 0
        self.show_failed     = False
        self.show_candidates = False
        self.show_chosen     = True
        self.show_unassembled_printed = False

        # ---- edit mode ----
        # A grasp is held as an offset in the part's own frame.  The pick and the
        # place are that one offset evaluated at the bay pose and at the assembled
        # pose, so moving either handle is the same act: it edits the offset, and
        # both ends follow.  That is why there is no separate pick/place state.
        self.edit_mode     = False
        self.edit_offsets: dict[int, list[float]] = {}   # part_id -> [dx, dy, dz] mm
        self.edit_tools:   dict[int, str]         = {}   # part_id -> "vacuum" | "gripper"
        self.edit_widths:  dict[int, float]       = {}   # part_id -> jaw opening (mm)
        self.edit_angles:  dict[int, float]       = {}   # part_id -> jaw axis (rad)
        # What the planner chose, kept aside so a grasp can be put back.
        self.edit_defaults: dict[int, dict]       = {}
        self.edit_handles: list                   = []   # gizmos + preview markers
        self.edit_nodes:   dict                   = {}   # (part_id, end) -> handles
        self.on_edit_changed = None                      # set by the app
        self.show_jigs     = True
        self.show_background = True

        # A caller-supplied origin is held fixed for the whole session so that
        # loading a plan never shifts the world under the camera.  Only when
        # there is nothing else to anchor to (no machine geometry) does the
        # scene fall back to centring on this assembly.
        if self._fixed_origin is not None:
            self.origin = np.asarray(self._fixed_origin, dtype=np.float64)
        elif doc.parts:
            self.origin = np.array([
                np.mean([p.assembled_pose.x for p in doc.parts]) * MM,
                np.mean([p.assembled_pose.y for p in doc.parts]) * MM,
                np.mean([p.assembled_pose.z for p in doc.parts]) * MM,
            ])
        else:
            self.origin = np.zeros(3)

    # ---- edit state ----

    def init_edit_state(self) -> None:
        """Seed the per-part grasp offsets and tools from the plan, then apply any
        saved overrides on top."""
        self.edit_offsets.clear()
        self.edit_tools.clear()
        self.edit_widths.clear()
        self.edit_angles.clear()
        self.edit_defaults.clear()
        part_map = {p.part_id: p for p in self.doc.parts}

        for g in self.doc.grasps:
            if not g.chosen or g.part_id is None:
                continue
            part = part_map.get(g.part_id)
            if part is None:
                continue
            self.edit_offsets[g.part_id] = [g.x_mm - part.assembled_pose.x,
                                            g.y_mm - part.assembled_pose.y,
                                            g.z_mm - part.assembled_pose.z]
            self.edit_tools[g.part_id] = "vacuum"

        for pg in self.doc.ppg_grasps:
            part = part_map.get(pg.part_id)
            if part is None:
                continue
            self.edit_offsets[pg.part_id] = [pg.x_mm - part.assembled_pose.x,
                                             pg.y_mm - part.assembled_pose.y,
                                             pg.z_mm - part.assembled_pose.z]
            self.edit_tools[pg.part_id] = "gripper"
            self.edit_widths[pg.part_id] = pg.width_mm
            self.edit_angles[pg.part_id] = pg.angle_rad

        # Snapshot before any override is applied — this is what "reset" restores.
        for pid, off in self.edit_offsets.items():
            self.edit_defaults[pid] = {
                "offset": list(off),
                "tool":   self.edit_tools.get(pid, "vacuum"),
                "width":  self.edit_widths.get(pid),
                "angle":  self.edit_angles.get(pid),
            }

        for o in self.doc.grasp_overrides:
            if o.part_id not in self.edit_offsets:
                continue
            self.edit_offsets[o.part_id] = [o.dx_mm, o.dy_mm, o.dz_mm]
            self.edit_tools[o.part_id] = o.tool
            if o.width_mm > 0.0:
                self.edit_widths[o.part_id] = o.width_mm
            if o.angle_rad is not None:
                self.edit_angles[o.part_id] = o.angle_rad

    def current_overrides(self) -> list:
        """The edits as records, for saving."""
        from arms_planner_core.scene_document import GraspOverride
        return [
            GraspOverride(part_id=pid, tool=self.edit_tools.get(pid, "vacuum"),
                          dx_mm=o[0], dy_mm=o[1], dz_mm=o[2],
                          width_mm=self.edit_widths.get(pid, 0.0),
                          angle_rad=self.edit_angles.get(pid))
            for pid, o in sorted(self.edit_offsets.items())
        ]

    def nominal_width(self, part_id: int) -> float:
        """A sensible jaw opening for a part the planner never computed one for.

        Uses the part's smaller horizontal extent, which is the axis a two-finger
        gripper would naturally close along.
        """
        cached = self.edit_widths.get(part_id)
        if cached:
            return cached
        part = next((p for p in self.doc.parts if p.part_id == part_id), None)
        if part is None:
            return 20.0
        try:
            v, _ = self.doc.mesh_vertices_triangles(part)
            ext = (v.max(axis=0) - v.min(axis=0)) / MM
            return float(min(ext[0], ext[1]))
        except Exception:
            return 20.0

    def set_width(self, part_id: int, width_mm: float) -> None:
        self.edit_widths[part_id] = max(0.1, float(width_mm))
        self.rebuild_edit_scene()

    def set_angle(self, part_id: int, angle_rad: float) -> None:
        self.edit_angles[part_id] = float(angle_rad)
        self.rebuild_edit_scene()

    def nominal_angle(self, part_id: int) -> float:
        return float(self.edit_angles.get(part_id, 0.0))

    def reset_grasp(self, part_id: int) -> bool:
        """Put one grasp back to what the planner chose, discarding the edits."""
        d = self.edit_defaults.get(part_id)
        if d is None:
            return False
        self.edit_offsets[part_id] = list(d["offset"])
        self.edit_tools[part_id]   = d["tool"]
        if d["width"] is not None:
            self.edit_widths[part_id] = d["width"]
        else:
            self.edit_widths.pop(part_id, None)
        if d["angle"] is not None:
            self.edit_angles[part_id] = d["angle"]
        else:
            self.edit_angles.pop(part_id, None)
        self.rebuild_edit_scene()
        return True

    # ---- position helpers ----

    def _p(self, x_mm: float, y_mm: float, z_mm: float) -> tuple:
        return (x_mm * MM - self.origin[0],
                y_mm * MM - self.origin[1],
                z_mm * MM - self.origin[2])

    # ---- geometry loading ----

    def load_meshes(self) -> None:
        for part in self.doc.parts:
            try:
                verts, tris = self.doc.mesh_vertices_triangles(part)
            except Exception as e:
                print(f"[viewer] WARNING: mesh failed part {part.part_id}: {e}", file=sys.stderr)
                continue
            handle = self.server.scene.add_mesh_simple(
                name=f"parts/part_{part.part_id}",
                vertices=verts.astype(np.float32),
                faces=tris.astype(np.uint32),
                flat_shading=True,
                color=_OPAQUE.get(part.type, _OPAQUE["unknown"]),
                opacity=None,
                side="double",
                position=self._p(
                    part.assembled_pose.x,
                    part.assembled_pose.y,
                    part.assembled_pose.z,
                ),
            )
            self.part_handles[part.part_id] = handle

    def load_grasps(self) -> None:
        # Grasp x/y/z are world-frame positions computed with every part at
        # its *assembled* pose (debugGrasps() sets the full target assembly
        # before probing).  Keep each grasp alongside its record so
        # apply_stage() can re-anchor it to the part's current (assembled or
        # bay) position rather than leaving it fixed at the assembled-frame
        # spot even when that part isn't assembled yet.
        self.grasp_records: list[tuple[object, object]] = []

        # Tool geometry for the chosen grasps, loaded once and instanced.
        nozzle = None
        if self.doc.nozzle_file:
            try:
                nozzle = self.doc.nozzle_vertices_triangles()
            except Exception as e:
                print(f"[viewer] WARNING: nozzle mesh failed, using markers: {e}",
                      file=sys.stderr)

        for g in self.doc.grasps:
            pos = self._p(g.x_mm, g.y_mm, g.z_mm)

            if g.chosen and self.doc.tools.get("vacuum") is not None:
                # Real tool model, placed by its own tip.
                tv = self.doc.tools["vacuum"]
                for h in self._draw_tool(f"grasps/chosen/tool_{id(g)}", "vacuum",
                                         (g.x_mm, g.y_mm, g.z_mm)):
                    self.grasp_handles_chosen.append(h)
                    # Placed at the cup's tip, which is below the recorded grasp
                    # point — remember where, so re-anchoring keeps the offset.
                    self.grasp_records.append(
                        (h, g, (g.x_mm, g.y_mm, g.z_mm + tv.origin_dz_mm)))
                group = "chosen"
                handle = None
            elif g.chosen and nozzle is not None:
                # A grasp's (x_mm, y_mm, z_mm) IS the nozzle centroid, and the
                # baked mesh is local-frame about that centroid, so the tool
                # lands exactly where the planner tested it — no offset needed.
                verts, tris = nozzle
                handle = self.server.scene.add_mesh_simple(
                    name=f"grasps/chosen/{id(g)}",
                    vertices=verts,
                    faces=tris,
                    flat_shading=True,
                    color=_NOZZLE_COLOUR,
                    opacity=None,
                    side="double",
                    position=pos,
                )
                group = "chosen"
            else:
                if g.chosen:
                    group, colour, radius = "chosen", _GRASP_CHOSEN_COLOUR, _GRASP_CHOSEN_RADIUS
                elif g.status == "accepted":
                    group, colour, radius = "candidate", _GRASP_COLOURS["accepted"], _GRASP_RADIUS
                else:
                    group  = "failed"
                    colour = _GRASP_COLOURS.get(g.status, (128, 128, 128))
                    radius = _GRASP_RADIUS
                handle = self.server.scene.add_icosphere(
                    name=f"grasps/{group}/{id(g)}",
                    radius=radius,
                    color=colour,
                    position=pos,
                )
            if handle is None:
                continue            # the tool model already registered its handles
            getattr(self, f"grasp_handles_{group}").append(handle)
            self.grasp_records.append((handle, g, (g.x_mm, g.y_mm, g.z_mm)))

    @staticmethod
    def _linkage_phi(tool, width_mm: float) -> float:
        """Arm angle for a given jaw opening, in the tool's x-z plane.

        The four-bar closes to the same opening either way — the arms can swing up
        or down for the same x — so the branch has to be chosen, not derived.  This
        mechanism folds *downward*: closing the jaws reaches them further below the
        body, rather than retracting them up into it.  Hence the negative angle.
        """
        if not (tool is not None and tool.has_linkage and width_mm):
            return 0.0
        L = tool.arm_length_mm
        rest_h = 0.5 * (tool.jaw_lo_inner_mm + tool.jaw_hi_inner_mm)
        return -float(np.arccos(np.clip(1.0 + (0.5 * width_mm - rest_h) / L, -1.0, 1.0)))

    @staticmethod
    def _linkage_lift(tool, width_mm: float) -> float:
        """Signed height the jaws move at this opening, relative to as-modelled.

        Negative as the jaws close, since they swing down — so the tool is raised
        by the same amount for the faces to end up where the plan asked.
        """
        if not (tool is not None and tool.has_linkage and width_mm):
            return 0.0
        return tool.arm_length_mm * float(np.sin(_ViewerState._linkage_phi(tool, width_mm)))

    def _draw_tool(self, name: str, tool_key: str, world_mm: tuple,
                   angle_rad: float = 0.0, width_mm: float | None = None) -> list:
        """Draw a baked end-effector model at a grasp, returning its handles.

        The tool's own origin is the point the grasp refers to, so placement is
        just that point plus the model's recorded drop.  A gripper is drawn from
        three pieces: the body, and the two jaws slid along the opening axis to
        the width the plan calls for.  The jaw translation is baked into the
        vertices because viser applies a handle's rotation before its position, so
        a local offset cannot be expressed through `position` alone.

        The baked models stand upright with their jaws separating along their own
        x, which is the axis a grasp angle is measured from — so the grasp angle
        is applied directly.
        """
        made: list = []
        tool = self.doc.tools.get(tool_key)
        if tool is None:
            return made

        half = 0.5 * angle_rad
        wxyz = (float(np.cos(half)), 0.0, 0.0, float(np.sin(half)))
        pos = self._p(world_mm[0], world_mm[1], world_mm[2] + tool.origin_dz_mm)

        def add(entry: str, dx_mm: float = 0.0, dz_mm: float = 0.0,
                key: str = "", spin: tuple | None = None) -> None:
            try:
                v, t = self.doc.tool_mesh(entry)
            except Exception as e:
                print(f"[viewer] WARNING: tool mesh {entry} failed: {e}", file=sys.stderr)
                return
            if spin is not None:
                # Turn about a hinge in the tool's own x-z plane.  Baked into the
                # vertices because a handle carries only one rotation, which is
                # already spent on the grasp angle.
                a, px, pz = spin
                v = v.copy()
                x = v[:, 0] - px * MM
                z = v[:, 2] - pz * MM
                v[:, 0] = x * np.cos(a) - z * np.sin(a) + px * MM
                v[:, 2] = x * np.sin(a) + z * np.cos(a) + pz * MM
            if dx_mm or dz_mm:
                v = v.copy()
                v[:, 0] += dx_mm * MM
                v[:, 2] += dz_mm * MM
            made.append(self.server.scene.add_mesh_simple(
                name=f"{name}_{key or 'body'}",
                vertices=v.astype(np.float32), faces=t,
                flat_shading=True, color=_JAW_COLOUR, opacity=None,
                side="double", position=pos, wxyz=wxyz))

        add(tool.body_file)
        if not (tool.has_jaws and width_mm):
            return made

        half_w   = 0.5 * width_mm
        rest_h   = 0.5 * (tool.jaw_lo_inner_mm + tool.jaw_hi_inner_mm)

        if not tool.has_linkage:
            # No linkage information: slide the jaws straight in.
            add(tool.jaw_lo_file, dx_mm=(tool.jaw_lo_inner_mm - half_w), key="jaw_lo")
            add(tool.jaw_hi_file, dx_mm=(half_w - tool.jaw_hi_inner_mm), key="jaw_hi")
            return made

        # Solve the four-bar.  Each jaw hangs on arms of length L hinged to the
        # chassis; with the arms horizontal the jaw sits at `rest_h`.  Swinging
        # them by phi moves the jaw face to L*cos(phi) and its height by
        # L*sin(phi) — so a tighter grip also reaches further down.
        L   = tool.arm_length_mm
        phi = self._linkage_phi(tool, width_mm)
        dz  = L * float(np.sin(phi))
        dx  = half_w - rest_h            # same for both, mirrored by sign

        add(tool.jaw_lo_file, dx_mm=-dx, dz_mm=dz, key="jaw_lo")
        add(tool.jaw_hi_file, dx_mm=+dx, dz_mm=dz, key="jaw_hi")

        # The arms rotate about their own hinge rather than translating, so the
        # whole mechanism stays connected instead of the jaws sliding free of it.
        for i, arm in enumerate(tool.arms):
            add(arm.file, key=f"arm{i}",
                spin=(arm.sign * phi, arm.pivot_x_mm, arm.pivot_z_mm))
        return made

    def load_jaws(self) -> None:
        """Draw the two gripper plates for every part picked with the parallel jaws.

        Built in the grasp frame — jaw axis along local x — and rotated about z by
        the grasp angle, which is the gripper's only rotational freedom.  They join
        the chosen-grasp group so the same toggle shows jaws and nozzles together.
        """
        for g in self.doc.ppg_grasps:
            tool = self.doc.tools.get("gripper")
            if tool is not None:
                # The model lines up on the jaw underside, not the tool's lowest
                # point, so back off by where its jaws start.
                z = (g.jaw_bottom_z_mm - tool.jaw_bottom_dz_mm
                     - self._linkage_lift(tool, g.width_mm))
                handles = self._draw_tool(f"grasps/chosen/tool_{g.part_id}", "gripper",
                                          (g.x_mm, g.y_mm, z),
                                          angle_rad=g.angle_rad, width_mm=g.width_mm)
                for h in handles:
                    self.grasp_handles_chosen.append(h)
                    # Record where the handle really sits, not the grasp centre:
                    # a tool model is offset from it by its own origin, and
                    # re-anchoring must preserve that.
                    self.jaw_records.append((h, g, (g.x_mm, g.y_mm, z)))
                continue

            half_w = 0.5 * g.width_mm
            z0 = g.jaw_bottom_z_mm - g.z_mm            # jaw underside, relative to centre
            z1 = z0 + g.jaw_height_mm
            half_face = 0.5 * g.jaw_width_mm

            # Rotation about +z only.
            half = 0.5 * g.angle_rad
            wxyz = (float(np.cos(half)), 0.0, 0.0, float(np.sin(half)))
            pos = self._p(g.x_mm, g.y_mm, g.z_mm)

            for side in (+1, -1):
                a0 = half_w if side > 0 else -half_w - g.jaw_thickness_mm
                v, t = _box_mesh((a0, -half_face, z0),
                                 (a0 + g.jaw_thickness_mm, half_face, z1))
                handle = self.server.scene.add_mesh_simple(
                    name=f"grasps/chosen/jaw_{g.part_id}_{'p' if side > 0 else 'm'}",
                    vertices=(v * MM).astype(np.float32),
                    faces=t,
                    flat_shading=True,
                    color=_JAW_COLOUR,
                    opacity=None,
                    side="double",
                    position=pos,
                    wxyz=wxyz,
                )
                self.grasp_handles_chosen.append(handle)
                self.jaw_records.append((handle, g, (g.x_mm, g.y_mm, g.z_mm)))

    # ---- edit mode rendering ----

    def _tool_preview(self, name: str, part_id: int, world_mm: tuple) -> list:
        """Draw the end effector at one end of a grasp, in whichever tool is selected.

        Returns the handles so they can be repositioned during a drag rather than
        rebuilt — recreating the gizmo mid-drag would pull it from under the cursor.
        """
        made: list = []
        tool = self.edit_tools.get(part_id, "vacuum")
        pos  = self._p(*world_mm)

        tm = self.doc.tools.get("gripper" if tool == "gripper" else "vacuum")
        if tm is not None:
            if tool == "gripper":
                width = self.edit_widths.get(part_id) or self.nominal_width(part_id)
                z = world_mm[2] - 0.5 * (self.doc.ppg_grasps[0].jaw_height_mm
                                         if self.doc.ppg_grasps else 12.0)
                return self._draw_tool(name, "gripper",
                                       (world_mm[0], world_mm[1],
                                        z - tm.jaw_bottom_dz_mm
                                        - self._linkage_lift(tm, width)),
                                       angle_rad=self.edit_angles.get(part_id, 0.0),
                                       width_mm=width)
            return self._draw_tool(name, "vacuum", world_mm)

        if tool == "gripper":
            pg = next((p for p in self.doc.ppg_grasps if p.part_id == part_id), None)
            # A part the planner picked with the cup has no jaw pose, so fall back
            # to nominal jaws sized from the part — enough to judge placement by.
            width = self.edit_widths.get(part_id) or (pg.width_mm if pg else self.nominal_width(part_id))
            jw    = pg.jaw_width_mm if pg else 10.0
            jh    = pg.jaw_height_mm if pg else 12.0
            jt    = pg.jaw_thickness_mm if pg else 4.0
            angle = self.edit_angles.get(part_id, pg.angle_rad if pg else 0.0)
            half  = 0.5 * angle
            wxyz  = (float(np.cos(half)), 0.0, 0.0, float(np.sin(half)))
            for side in (+1, -1):
                a0 = 0.5 * width if side > 0 else -0.5 * width - jt
                v, t = _box_mesh((a0, -0.5 * jw, -0.5 * jh), (a0 + jt, 0.5 * jw, 0.5 * jh))
                made.append(self.server.scene.add_mesh_simple(
                    name=f"{name}_jaw{'p' if side > 0 else 'm'}",
                    vertices=(v * MM).astype(np.float32), faces=t,
                    flat_shading=True, color=_JAW_COLOUR, opacity=None,
                    side="double", position=pos, wxyz=wxyz))
        else:
            try:
                verts, tris = self.doc.nozzle_vertices_triangles()
                made.append(self.server.scene.add_mesh_simple(
                    name=f"{name}_nozzle", vertices=verts, faces=tris,
                    flat_shading=True, color=_NOZZLE_COLOUR, opacity=None,
                    side="double", position=pos))
            except Exception:
                made.append(self.server.scene.add_icosphere(
                    name=f"{name}_nozzle", radius=_GRASP_CHOSEN_RADIUS,
                    color=_GRASP_CHOSEN_COLOUR, position=pos))

        return made

    def rebuild_edit_scene(self) -> None:
        """(Re)draw the editable grasps: a gizmo and a tool preview at both the pick
        and the place position of every grasp.

        Both ends are shown at once, whatever stage is selected — the point of edit
        mode is to see the pick and the place together, since one offset drives them
        both."""
        for h in self.edit_handles:
            try:
                h.remove()
            except Exception:
                pass
        self.edit_handles.clear()
        if not self.edit_mode:
            return

        part_map = {p.part_id: p for p in self.doc.parts}

        self.edit_nodes = {}

        for part_id, off in self.edit_offsets.items():
            part = part_map.get(part_id)
            if part is None:
                continue

            for end, base in (("pick", part.bay_pose), ("place", part.assembled_pose)):
                world = (base.x + off[0], base.y + off[1], base.z + off[2])
                tool_handles = self._tool_preview(f"edit/{part_id}/{end}", part_id, world)

                ctrl = self.server.scene.add_transform_controls(
                    name=f"edit/{part_id}/{end}/gizmo",
                    scale=0.05,
                    disable_rotations=True,
                    position=self._p(*world),
                )
                self.edit_handles.extend(tool_handles)
                self.edit_handles.append(ctrl)
                self.edit_nodes[(part_id, end)] = {"gizmo": ctrl, "tools": tool_handles,
                                                   "base": base}

                @ctrl.on_update
                def _(handle, _pid=part_id, _end=end, _base=base) -> None:
                    x, y, z = handle.position
                    self.edit_offsets[_pid] = [
                        (x + self.origin[0]) / MM - _base.x,
                        (y + self.origin[1]) / MM - _base.y,
                        (z + self.origin[2]) / MM - _base.z,
                    ]
                    # Move everything else to match.  The dragged gizmo is left
                    # alone so it stays under the cursor.
                    self.sync_edit_positions(_pid, skip_end=_end)
                    if self.on_edit_changed:
                        self.on_edit_changed()

    def sync_edit_positions(self, part_id: int, skip_end: str | None = None) -> None:
        """Reposition a grasp's handles from its current offset.

        The pick and the place are the same offset at two different part poses, so
        editing one end necessarily moves the other; this is where that happens.
        """
        off = self.edit_offsets.get(part_id)
        if off is None:
            return
        for end in ("pick", "place"):
            node = getattr(self, "edit_nodes", {}).get((part_id, end))
            if node is None:
                continue
            base = node["base"]
            pos = self._p(base.x + off[0], base.y + off[1], base.z + off[2])
            for h in node["tools"]:
                h.position = pos
            if end != skip_end:
                node["gizmo"].position = pos

    def set_tool(self, part_id: int, tool: str) -> None:
        self.edit_tools[part_id] = tool
        self.rebuild_edit_scene()

    def load_jigs(self, arms_dir: str) -> None:
        jig_offset = tuple((-self.origin).tolist())
        for jig in self.doc.jigs:
            stl_path = os.path.join(arms_dir, jig.stl_file)
            if not os.path.exists(stl_path):
                print(f"[viewer] WARNING: jig STL not found: {stl_path}", file=sys.stderr)
                continue
            try:
                with open(stl_path, 'rb') as f:
                    data = f.read()
                verts_mm, tris = _parse_stl(data)
                handle = self.server.scene.add_mesh_simple(
                    name=f"jigs/{jig.stl_file}",
                    vertices=(verts_mm * MM).astype(np.float32),
                    faces=tris.astype(np.uint32),
                    flat_shading=True,
                    color=_JIG_COLOUR,
                    opacity=None,
                    side="double",
                    position=jig_offset,
                )
                self.jig_handles.append(handle)
            except Exception as e:
                print(f"[viewer] WARNING: jig failed {stl_path}: {e}", file=sys.stderr)

    def load_backgrounds(self) -> None:
        """Add the static environment meshes baked into the .arms file.

        These GLBs are written in world-space metres (arms-plan bakes the pose
        from background_models.txt into the vertices), so they take the same
        flat scene-centring offset as jigs rather than a per-object position.
        """
        bg_offset = tuple((-self.origin).tolist())
        for i, bg in enumerate(self.doc.background):
            try:
                verts, tris = self.doc.background_vertices_triangles(bg)
            except Exception as e:
                print(f"[viewer] WARNING: background mesh failed {bg.file}: {e}",
                      file=sys.stderr)
                continue
            if verts.size == 0 or tris.size == 0:
                print(f"[viewer] WARNING: background mesh empty: {bg.file}",
                      file=sys.stderr)
                continue
            handle = self.server.scene.add_mesh_simple(
                name=f"background/{i}_{bg.file}",
                vertices=verts.astype(np.float32),
                faces=tris.astype(np.uint32),
                flat_shading=True,
                color=bg.color_rgb255,
                opacity=None,
                side="double",
                position=bg_offset,
            )
            self.background_handles.append(handle)

    # ---- stage update ----

    def apply_stage(self, stage_idx: int) -> None:
        if not self.doc.stages:
            return
        stage_idx = max(0, min(stage_idx, self.n_stages - 1))
        self.current_stage = stage_idx
        stage = self.doc.stages[stage_idx]
        assembled   = set(stage.assembled_part_ids)
        unassembled = set(stage.unassembled_part_ids)
        part_map    = {p.part_id: p for p in self.doc.parts}

        for part_id, handle in self.part_handles.items():
            cb = self.part_checkboxes.get(part_id)
            if cb is not None and not cb.value:
                handle.visible = False
                continue
            part = part_map.get(part_id)
            if part is None:
                handle.visible = False
                continue

            if part_id in assembled:
                handle.position = self._p(
                    part.assembled_pose.x, part.assembled_pose.y, part.assembled_pose.z)
                handle.color   = _OPAQUE.get(part.type, _OPAQUE["unknown"])
                handle.opacity = None
                handle.visible = True
            elif part_id in unassembled:
                if part.type in _HIDE_WHEN_UNASSEMBLED:
                    handle.visible = False
                    continue
                if (part.type in _OPTIONAL_WHEN_UNASSEMBLED
                        and not self.show_unassembled_printed):
                    handle.visible = False
                    continue
                handle.position = self._p(
                    part.bay_pose.x, part.bay_pose.y, part.bay_pose.z)
                handle.color   = _UNASSEMBLED.get(part.type, _UNASSEMBLED["unknown"])
                handle.opacity = None
                handle.visible = True
            else:
                handle.visible = False

        # Re-anchor each grasp marker to wherever its part currently is
        # (assembled or bay position) rather than leaving it fixed at the
        # assembled-frame position it was computed at.
        for handle, g, world in self.grasp_records:
            if g.part_id is None:
                continue  # legacy .arms file written before grasps carried part_id
            part = part_map.get(g.part_id)
            if part is None:
                continue

            if g.part_id in assembled:
                base = part.assembled_pose
            elif g.part_id in unassembled:
                base = part.bay_pose
            else:
                continue

            offset_mm = (world[0] - part.assembled_pose.x,
                         world[1] - part.assembled_pose.y,
                         world[2] - part.assembled_pose.z)
            handle.position = self._p(
                base.x + offset_mm[0], base.y + offset_mm[1], base.z + offset_mm[2])

        self._reanchor_jaws(assembled, unassembled, part_map)

    def _reanchor_jaws(self, assembled: set, unassembled: set, part_map: dict) -> None:
        """Move the jaw plates with their part, exactly as grasp markers move."""
        for handle, g, world in self.jaw_records:
            part = part_map.get(g.part_id)
            if part is None:
                continue
            if g.part_id in assembled:
                base = part.assembled_pose
            elif g.part_id in unassembled:
                base = part.bay_pose
            else:
                continue
            handle.position = self._p(
                base.x + (world[0] - part.assembled_pose.x),
                base.y + (world[1] - part.assembled_pose.y),
                base.z + (world[2] - part.assembled_pose.z))

    def _apply_grasp_visibility(self) -> None:
        for h in self.grasp_handles_failed:
            h.visible = self.show_failed
        for h in self.grasp_handles_candidate:
            h.visible = self.show_candidates
        for h in self.grasp_handles_chosen:
            h.visible = self.show_chosen

    def _apply_jig_visibility(self) -> None:
        for h in self.jig_handles:
            h.visible = self.show_jigs

    def _apply_background_visibility(self) -> None:
        for h in self.background_handles:
            h.visible = self.show_background


# ---------------------------------------------------------------------------
# Planning log → status line
#
# arms-plan writes two kinds of lines to stdout/stderr:
#   "[arms-plan] ..."          coarse top-level milestones (main.cpp)
#   "[arms/INFO|WARN|FATAL] ..." fine-grained internal logging (Logger.hpp)
# The status line only moves on recognised milestones, per-part progress
# markers, and warnings/errors — everything else is internal noise that
# still lands in the raw history buffer for the "show history" toggle.
# ---------------------------------------------------------------------------

_MAX_HISTORY_LINES = 500

_PART_PROGRESS: list[tuple[re.Pattern, str]] = [
    (re.compile(r"VacuumGraspGenerator: grasp for (\S+)"),
     "🦾 Found grasp for {0}"),
    (re.compile(r"generateGrasps: no grasp found for (\S+)"),
     "⚠️  No grasp found for {0}"),
    (re.compile(r"(?:JigGenerator|AdvancedCradleGenerator): (\S+)\s+bay="),
     "🖨️  Generating jig for {0}"),
    (re.compile(r"SparseJigGenerator::buildJigShape: (\S+)\s+bay="),
     "🖨️  Generating jig for {0}"),
]

_MILESTONES: list[tuple[re.Pattern, str]] = [
    (re.compile(r"^\[arms-plan\] Loading model"),               "📄 Loading STEP file…"),
    (re.compile(r"^\[arms-plan\] Running pipeline"),             "🧩 Computing assembly sequence…"),
    (re.compile(r"Generating Slicer Gcode for internal parts"),  "🍞 Slicing print-bed g-code…"),
    (re.compile(r"^\[arms-plan\] Collecting grasp debug data"),  "🦾 Analysing grasp candidates…"),
    (re.compile(r"^\[arms-plan\] Building manifest"),            "📋 Building manifest…"),
    (re.compile(r"^\[arms-plan\] Writing "),                     "💾 Writing output file…"),
    (re.compile(r"^\[arms-plan\] Done\."),                       "✅ Done"),
]

_WARN_RE  = re.compile(r"^\[arms/WARN\]\s*(.*)")
_FATAL_RE = re.compile(r"^\[arms/(?:FATAL|ERROR)\]\s*(.*)")


def _friendly_line(raw: str) -> str | None:
    """Simplify a raw arms-plan log line for the status display, or return
    None if it's internal noise that shouldn't move the display (it's still
    kept in the raw history)."""
    for pattern, template in _PART_PROGRESS:
        m = pattern.search(raw)
        if m:
            return template.format(m.group(1))
    for pattern, label in _MILESTONES:
        if pattern.search(raw):
            return label
    m = _FATAL_RE.search(raw)
    if m:
        return f"❌ {m.group(1)}"
    m = _WARN_RE.search(raw)
    if m:
        return f"⚠️  {m.group(1)}"
    return None


# ---------------------------------------------------------------------------
# Integrated application
# ---------------------------------------------------------------------------

class ARMSPlannerApp:
    """Manages the full planning + viewing UI within a viser server."""

    def __init__(self, step_dir: str, server: viser.ViserServer):
        self.step_dir = Path(step_dir).resolve()
        self.server   = server

        self._viewer_state: _ViewerState | None = None
        self._planning   = False

        # Raw planner log lines from the most recent run, for the
        # "show message history" toggle.
        self._log_history: list[str] = []
        self._show_history = False

        # Scene-specific GUI handles — cleared on each scene load.
        self._stage_children: list = []
        self._parts_children:  list = []

        # Persistent background.  Owned by the app rather than _ViewerState so it
        # survives scene.reset() and is visible with no plan loaded.  Geometry is
        # parsed once; only the handles are recreated after a reset.
        self._bg_meshes: list[tuple] = _load_background_models()
        self._bg_handles: list = []
        self._show_background = True

        # The scene origin is decided once and never changes, so nothing ever
        # shifts under the camera.  With machine geometry we anchor on that;
        # without it there is nothing to show until a plan loads, so we defer
        # (None) and adopt the first plan's assembly centroid, then freeze.
        self._scene_origin: np.ndarray | None = (
            self._background_bbox_centre() if self._bg_meshes else None)
        if self._bg_meshes:
            self._camera_position = _CAMERA_POSITION_OVERVIEW
            self._camera_look_at  = _CAMERA_LOOK_AT_OVERVIEW
        else:
            self._camera_position = _CAMERA_POSITION
            self._camera_look_at  = _CAMERA_LOOK_AT

        # Live view of the assembly search, on its own port so it can be opened
        # in a browser tab and picked over while the planner is still running.
        self._search = SearchGraph()
        self._search_server = SearchGraphServer(
            self._search, viewer_url=f"http://localhost:{server.get_port()}/")
        self._search_ok = self._search_server.start()

        self._build_gui()
        self._ensure_background()

    # ---- persistent background ----

    def _background_bbox_centre(self) -> np.ndarray:
        """Centre of all background geometry, used to frame the machine when no
        plan is loaded (a plan's own assembly centroid takes over once one is)."""
        if not self._bg_meshes:
            return np.zeros(3)
        lo = np.min([v.min(axis=0) for v, _, _ in self._bg_meshes], axis=0)
        hi = np.max([v.max(axis=0) for v, _, _ in self._bg_meshes], axis=0)
        return (lo + hi) * 0.5

    def _ensure_background(self) -> None:
        """(Re)create the background handles — call after any scene.reset()."""
        self._bg_handles = []
        if not self._bg_meshes:
            return
        offset = tuple((-self._scene_origin).tolist())
        for i, (verts, tris, colour) in enumerate(self._bg_meshes):
            try:
                handle = self.server.scene.add_mesh_simple(
                    name=f"background/{i}",
                    vertices=verts,
                    faces=tris,
                    flat_shading=True,
                    color=colour,
                    opacity=None,
                    side="double",
                    position=offset,
                )
            except Exception as e:
                print(f"[viewer] WARNING: background mesh {i} failed: {e}", file=sys.stderr)
                continue
            handle.visible = self._show_background
            self._bg_handles.append(handle)

    def _apply_background_visibility(self) -> None:
        for h in self._bg_handles:
            h.visible = self._show_background

    # ---- file discovery ----

    def _step_files(self) -> list[Path]:
        results: set[Path] = set()
        for pat in ("*.step", "*.STEP", "*.stp", "*.STP"):
            results.update(self.step_dir.glob(pat))
        return sorted(results, key=lambda p: p.name.lower())

    def _arms_path(self, step_path: Path) -> Path:
        return self.step_dir / "arms_output" / step_path.stem / f"{step_path.stem}.arms"

    def _arms_dir(self, step_path: Path) -> Path:
        return self.step_dir / "arms_output" / step_path.stem

    def _current_step_path(self) -> Path | None:
        if not self._step_files():
            return None
        val = self._step_dd.value
        for s in self._step_files():
            if s.name == val:
                return s
        return self._step_files()[0]

    # ---- GUI construction ----

    def _build_gui(self) -> None:
        steps   = self._step_files()
        options = [s.name for s in steps] if steps else ["(no .step files found)"]

        # ---- Model selector (always visible) ----
        with self.server.gui.add_folder("Model"):
            self._step_dd   = self.server.gui.add_dropdown("STEP file", options=options)
            self._btn_plan  = self.server.gui.add_button("▶  Start Plan")
            self._btn_show  = self.server.gui.add_button("⚡  Show Plan")
            self._status_md = self.server.gui.add_markdown("_Select a model to begin._")
            self._history_cb = self.server.gui.add_checkbox(
                "Show message history", initial_value=False
            )
            self._history_md = self.server.gui.add_markdown("")

        # ---- Stage folder — placeholder replaced on load ----
        with self.server.gui.add_folder("Stage") as self._stage_folder:
            self._stage_children = [
                self.server.gui.add_markdown("_No model loaded._")
            ]

        # ---- Parts folder — placeholder replaced on load ----
        with self.server.gui.add_folder("Parts", expand_by_default=False) as self._parts_folder:
            # Deliberately not tracked in _parts_children: those are cleared and
            # rebuilt per scene, and this setting should persist across loads.
            self._show_unassembled_printed_cb = self.server.gui.add_checkbox(
                "Show unassembled printed parts", initial_value=False
            )
            self._parts_children = [
                self.server.gui.add_markdown("_No model loaded._")
            ]

        # ---- Grasps / Jigs (always visible) ----
        with self.server.gui.add_folder("Grasps", expand_by_default=False):
            self._show_chosen_cb = self.server.gui.add_checkbox(
                "Show chosen grasp", initial_value=True
            )
            self._show_candidates_cb = self.server.gui.add_checkbox(
                "Show successful candidates", initial_value=False
            )
            self._show_failed_cb = self.server.gui.add_checkbox(
                "Show failed candidates", initial_value=False
            )
        with self.server.gui.add_folder("Jigs", expand_by_default=False):
            self._show_jigs_cb = self.server.gui.add_checkbox(
                "Show jigs", initial_value=True
            )
            # Applied on the next plan — the pockets are cut by the planner, so
            # changing this only takes effect when jigs are regenerated.
            self._jig_clearance = self.server.gui.add_number(
                "Pocket clearance (mm)", 0.2, min=0.0, step=0.05
            )
        with self.server.gui.add_folder("Search", expand_by_default=False):
            # Shown in the panel itself, which is convenient but narrow.  The
            # standalone page below gets a whole window, which a search of any
            # size needs.
            self._search_plot = self.server.gui.add_plotly(
                build_figure(self._search.snapshot()), aspect=1.0
            )
            if self._search_ok:
                self._search_open_btn = self.server.gui.add_button(
                    "Open full-window search view", icon=viser.Icon.EXTERNAL_LINK
                )
                self._search_note = self.server.gui.add_markdown("")
            else:
                self._search_open_btn = None
                self._search_note = self.server.gui.add_markdown(
                    "_Standalone view unavailable — no free port._")

        with self.server.gui.add_folder("Edit", expand_by_default=False) as self._edit_folder:
            self._edit_mode_cb = self.server.gui.add_checkbox(
                "Edit grasps", initial_value=False
            )
            self._edit_name = self.server.gui.add_text(
                "Save as", initial_value="edited"
            )
            self._edit_save_btn  = self.server.gui.add_button("Save grasp edits")
            self._edit_apply_btn = self.server.gui.add_button("Apply edits → g-code")
            self._edit_status   = self.server.gui.add_markdown(
                "_Enable to drag grasps and switch tools._")
            self._edit_children: list = []

        with self.server.gui.add_folder("Background", expand_by_default=False):
            self._show_bg_cb = self.server.gui.add_checkbox(
                "Show background", initial_value=True
            )

        # ---- Callbacks ----

        @self._step_dd.on_update
        def _(_: viser.GuiEvent) -> None:
            self._refresh_status()

        @self._btn_plan.on_click
        def _(_: viser.GuiEvent) -> None:
            threading.Thread(target=self._do_plan, daemon=True).start()

        @self._btn_show.on_click
        def _(_: viser.GuiEvent) -> None:
            sp = self._current_step_path()
            if sp is None:
                return
            ap = self._arms_path(sp)
            if ap.exists():
                threading.Thread(
                    target=self._load_scene,
                    args=(ap, self._arms_dir(sp)),
                    daemon=True,
                ).start()
            else:
                self._status_md.content = (
                    "❌ No existing plan found — click **Start Plan** to generate one."
                )

        def _sync_grasp_visibility() -> None:
            if not self._viewer_state:
                return
            self._viewer_state.show_chosen     = bool(self._show_chosen_cb.value)
            self._viewer_state.show_candidates = bool(self._show_candidates_cb.value)
            self._viewer_state.show_failed     = bool(self._show_failed_cb.value)
            self._viewer_state._apply_grasp_visibility()

        for _cb in (self._show_chosen_cb, self._show_candidates_cb, self._show_failed_cb):
            _cb.on_update(lambda _: _sync_grasp_visibility())

        @self._show_jigs_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._viewer_state:
                self._viewer_state.show_jigs = bool(self._show_jigs_cb.value)
                self._viewer_state._apply_jig_visibility()

        if self._search_open_btn is not None:
            @self._search_open_btn.on_click
            def _(_: viser.GuiEvent) -> None:
                # Opened from the server side so it works the same whether the
                # viewer is running in the desktop window or a browser: the
                # desktop shell has no tabs of its own to open one in.
                import webbrowser
                url = self._search_server.url
                try:
                    webbrowser.open_new_tab(url)
                    self._search_note.content = f"_Opened <code>{url}</code>_"
                except Exception:
                    self._search_note.content = f"_Open manually: <code>{url}</code>_"

        @self._edit_mode_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            self._set_edit_mode(bool(self._edit_mode_cb.value))

        @self._edit_save_btn.on_click
        def _(_: viser.GuiEvent) -> None:
            self._save_grasp_edits()

        @self._edit_apply_btn.on_click
        def _(_: viser.GuiEvent) -> None:
            threading.Thread(target=self._apply_grasp_edits, daemon=True).start()

        @self._show_unassembled_printed_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._viewer_state:
                self._viewer_state.show_unassembled_printed = bool(
                    self._show_unassembled_printed_cb.value)
                self._viewer_state.apply_stage(self._viewer_state.current_stage)

        @self._show_bg_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            self._show_background = bool(self._show_bg_cb.value)
            self._apply_background_visibility()
            if self._viewer_state:
                self._viewer_state.show_background = self._show_background
                self._viewer_state._apply_background_visibility()

        @self._history_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            self._show_history = bool(self._history_cb.value)
            self._refresh_history_display()

        @self.server.on_client_connect
        def _(client: viser.ClientHandle) -> None:
            self._apply_default_camera(client)

        self._refresh_status()

    def _apply_default_camera(self, client: viser.ClientHandle | None = None) -> None:
        """Point a client's camera at the assembly.

        With no argument, applies to every currently-connected client — used on
        scene load, when the browser/webview is already attached.
        """
        targets = ([client] if client is not None
                   else list(self.server.get_clients().values()))
        for c in targets:
            try:
                c.camera.position = self._camera_position
                c.camera.look_at  = self._camera_look_at
            except Exception as e:
                print(f"[viewer] WARNING: could not set camera: {e}", file=sys.stderr)

    # ---- grasp editing ----

    def _set_edit_mode(self, on: bool) -> None:
        state = self._viewer_state
        if state is None:
            self._edit_status.content = "_Load a model first._"
            return

        state.edit_mode = on
        state.rebuild_edit_scene()
        self._build_edit_gui(state) if on else self._clear_edit_gui()

        # Planner-computed markers would sit on top of the editable ones, so they
        # step aside while editing and come back afterwards.
        state.show_chosen = (not on) and bool(self._show_chosen_cb.value)
        state._apply_grasp_visibility()

        self._edit_status.content = (
            f"_Dragging either end moves the other — {len(state.edit_offsets)} "
            f"grasp(s) editable._" if on
            else "_Enable to drag grasps and switch tools._")

    def _clear_edit_gui(self) -> None:
        for h in getattr(self, "_edit_children", []):
            try:
                h.remove()
            except Exception:
                pass
        self._edit_children = []

    def _selected_edit_part(self) -> int | None:
        state = self._viewer_state
        if state is None or not getattr(self, "_edit_part_dd", None):
            return None
        return self._edit_part_labels.get(str(self._edit_part_dd.value))

    def _sync_edit_fields(self) -> None:
        """Push the current offset and opening into the number fields.

        Called after a drag so the typed values track the handles.  Assigning to a
        viser input fires its own callback, so a guard stops that writing straight
        back into the state we just read.
        """
        state = self._viewer_state
        pid = self._selected_edit_part()
        if state is None or pid is None:
            return
        off = state.edit_offsets.get(pid)
        if off is None:
            return
        self._edit_syncing = True
        try:
            self._edit_x.value = round(float(off[0]), 3)
            self._edit_y.value = round(float(off[1]), 3)
            self._edit_z.value = round(float(off[2]), 3)
            self._edit_tool_dd.value = state.edit_tools.get(pid, "vacuum")
            self._edit_width.value = round(float(state.nominal_width(pid)), 2)
            self._edit_angle.value = round(float(state.nominal_angle(pid)) * 180.0 / np.pi, 1)
            gripper = state.edit_tools.get(pid) == "gripper"
            self._edit_width.disabled = not gripper
            self._edit_angle.disabled = not gripper
        finally:
            self._edit_syncing = False

    def _build_edit_gui(self, state: _ViewerState) -> None:
        """Controls for one grasp at a time.

        A single panel that follows a selection, rather than a row per part: the
        offset alone is three numbers, and with the tool and the jaw opening that
        would be five controls per part.
        """
        self._clear_edit_gui()
        part_map = {p.part_id: p for p in state.doc.parts}
        self._edit_syncing = False

        labels = {}
        for part_id in sorted(state.edit_offsets):
            part = part_map.get(part_id)
            if part is not None:
                labels[part.name] = part_id
        self._edit_part_labels = labels
        if not labels:
            return

        with self._edit_folder:
            self._edit_part_dd = self.server.gui.add_dropdown(
                "Grasp", options=tuple(labels.keys()), initial_value=next(iter(labels))
            )
            self._edit_tool_dd = self.server.gui.add_dropdown(
                "Tool", options=("vacuum", "gripper"), initial_value="vacuum"
            )
            # Offsets are in the part's own frame, so the same three numbers
            # describe the pick and the place.
            self._edit_x = self.server.gui.add_number("Offset X (mm)", 0.0, step=0.1)
            self._edit_y = self.server.gui.add_number("Offset Y (mm)", 0.0, step=0.1)
            self._edit_z = self.server.gui.add_number("Offset Z (mm)", 0.0, step=0.1)
            self._edit_width = self.server.gui.add_number(
                "Jaw opening (mm)", 20.0, min=0.1, step=0.1
            )
            # Degrees in the interface, radians in the plan — the gripper's one
            # rotational freedom, about the build axis.
            self._edit_angle = self.server.gui.add_number(
                "Jaw angle (deg)", 0.0, step=1.0
            )
            self._edit_reset_btn = self.server.gui.add_button("Reset to planner")
            self._edit_nudge = self.server.gui.add_number("Nudge step (mm)", 1.0, min=0.01, step=0.1)

        self._edit_children = [self._edit_part_dd, self._edit_tool_dd,
                               self._edit_x, self._edit_y, self._edit_z,
                               self._edit_width, self._edit_angle,
                               self._edit_reset_btn, self._edit_nudge]

        def apply_numbers(_: viser.GuiEvent) -> None:
            if self._edit_syncing:
                return
            pid = self._selected_edit_part()
            if pid is None:
                return
            state.edit_offsets[pid] = [float(self._edit_x.value),
                                       float(self._edit_y.value),
                                       float(self._edit_z.value)]
            state.sync_edit_positions(pid)
            self._edit_status.content = (
                f"_Offset set to ({self._edit_x.value:.3f}, {self._edit_y.value:.3f}, "
                f"{self._edit_z.value:.3f}) mm._")

        for fld in (self._edit_x, self._edit_y, self._edit_z):
            fld.on_update(apply_numbers)

        @self._edit_part_dd.on_update
        def _(_: viser.GuiEvent) -> None:
            self._sync_edit_fields()

        @self._edit_tool_dd.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._edit_syncing:
                return
            pid = self._selected_edit_part()
            if pid is None:
                return
            state.set_tool(pid, str(self._edit_tool_dd.value))
            self._sync_edit_fields()
            self._edit_status.content = f"_{self._edit_part_dd.value} → {self._edit_tool_dd.value}._"

        @self._edit_width.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._edit_syncing:
                return
            pid = self._selected_edit_part()
            if pid is None:
                return
            state.set_width(pid, float(self._edit_width.value))
            self._edit_status.content = f"_Jaw opening {self._edit_width.value:.2f} mm._"

        @self._edit_angle.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._edit_syncing:
                return
            pid = self._selected_edit_part()
            if pid is None:
                return
            state.set_angle(pid, float(self._edit_angle.value) * np.pi / 180.0)
            self._edit_status.content = f"_Jaw angle {self._edit_angle.value:.1f}°._"

        @self._edit_reset_btn.on_click
        def _(_: viser.GuiEvent) -> None:
            pid = self._selected_edit_part()
            if pid is None:
                return
            if state.reset_grasp(pid):
                self._sync_edit_fields()
                self._edit_status.content = (
                    f"_{self._edit_part_dd.value} reset to the planner's grasp._")
            else:
                self._edit_status.content = "_Nothing to reset._"

        # Keep the typed values in step with dragging.
        state.on_edit_changed = self._sync_edit_fields
        self._sync_edit_fields()

    def _save_grasp_edits(self) -> None:
        state = self._viewer_state
        if state is None:
            return
        try:
            state.doc.write_overrides(state.current_overrides())
        except Exception as e:
            self._edit_status.content = f"❌ Could not save: {e}"
            return
        self._edit_status.content = (
            f"✅ Saved {len(state.edit_offsets)} grasp edit(s) into the .arms file._")

    def _apply_grasp_edits(self) -> None:
        """Re-run the planner with the edited grasps so the g-code carries them.

        The grasps drive the pick and place moves, the tool changes and the command
        file, all of which are generated in the C++ pipeline — so the edits are fed
        back through it rather than patched into the output here.  The sequence is
        untouched: the user changed how a part is picked, not the order.
        """
        state = self._viewer_state
        sp = self._current_step_path()
        if state is None or sp is None or not sp.exists():
            self._edit_status.content = "❌ Load a model first."
            return
        if self._planning:
            self._edit_status.content = "⏳ A plan is already running."
            return

        name = (str(self._edit_name.value).strip() or "edited")
        name = re.sub(r"[^A-Za-z0-9._-]+", "_", name)

        out_dir  = self._arms_dir(sp).parent / f"{sp.stem}_{name}"
        out_arms = out_dir / f"{sp.stem}_{name}.arms"
        out_dir.mkdir(parents=True, exist_ok=True)

        overrides_path = out_dir / "grasp_overrides.json"
        overrides_path.write_text(json.dumps([
            {"part_id": o.part_id, "tool": o.tool,
             "dx_mm": o.dx_mm, "dy_mm": o.dy_mm, "dz_mm": o.dz_mm,
             "width_mm": o.width_mm}
            for o in state.current_overrides()
        ], indent=2))

        self._planning = True
        self._edit_status.content = f"⏳ Regenerating **{out_arms.name}**…"
        cmd = [
            "arms-plan",
            "--input",            str(sp),
            "--output",           str(out_arms),
            "--output-dir",       str(out_dir) + "/",
            "--grasp-overrides",  str(overrides_path),
        ] + _plan_common_args(float(self._jig_clearance.value))
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, bufsize=1, cwd=str(self.step_dir))
            for raw in proc.stdout:
                line = raw.rstrip()
                if not line:
                    continue
                if self._search.feed(line):
                    self._refresh_search_plot()
                    continue
                self._record_log_line(line)
                nice = _friendly_line(line)
                if nice:
                    self._status_md.content = nice
            proc.wait()
            self._search.finish()
            self._refresh_search_plot(force=True)
        except FileNotFoundError:
            self._edit_status.content = "❌ `arms-plan` not on PATH."
            self._planning = False
            return
        finally:
            self._planning = False

        if proc.returncode != 0 or not out_arms.exists():
            self._edit_status.content = (
                f"❌ Regeneration failed (exit {proc.returncode}) — see message history."
            )
            return

        # Loading resets edit mode, which clears the status — so report after it.
        self._load_scene(out_arms, out_dir)
        self._edit_status.content = (
            f"✅ Wrote **{out_arms.name}** and its g-code with your edits applied."
        )
        self._status_md.content = f"✅ Regenerated **{out_arms.stem}**"

    def _refresh_search_plot(self, force: bool = False) -> None:
        """Redraw the embedded search graph, at most a few times a second.

        Events arrive far faster than the panel can usefully update, and each
        redraw rebuilds the whole figure — so this is throttled rather than tied
        to the event rate.
        """
        now = time.time()
        if not force and now - getattr(self, "_search_plot_at", 0.0) < 0.7:
            return
        self._search_plot_at = now
        try:
            fig = build_figure(self._search.snapshot())
            if fig is not None:
                self._search_plot.figure = fig
        except Exception:
            pass        # the graph is a diagnostic; never let it break a plan

    def _refresh_history_display(self) -> None:
        if not self._show_history or not self._log_history:
            self._history_md.content = ""
            return
        lines = self._log_history[-_MAX_HISTORY_LINES:]
        self._history_md.content = "\n\n".join(f"`{l}`" for l in lines)

    def _record_log_line(self, line: str) -> None:
        self._log_history.append(line)
        # Bound memory on pathologically verbose runs (e.g. large DFS searches)
        # while always keeping the most recent _MAX_HISTORY_LINES.
        if len(self._log_history) > _MAX_HISTORY_LINES * 4:
            del self._log_history[: -_MAX_HISTORY_LINES * 2]
        if self._show_history:
            self._refresh_history_display()

    def _refresh_status(self) -> None:
        sp = self._current_step_path()
        if sp is None:
            self._status_md.content = "_No .step files found in the working directory._"
            return
        ap = self._arms_path(sp)
        if ap.exists():
            self._status_md.content = (
                f"**{sp.stem}** — existing plan found.  \n"
                "Click **Show Plan** to view, or **Start Plan** to re-plan."
            )
        else:
            self._status_md.content = (
                f"**{sp.stem}** — no plan yet.  \n"
                "Click **Start Plan** to generate."
            )

    # ---- planning subprocess ----

    def _do_plan(self) -> None:
        if self._planning:
            return
        sp = self._current_step_path()
        if sp is None or not sp.exists():
            self._status_md.content = f"❌ File not found: {self._step_dd.value}"
            return

        self._planning = True
        self._clear_scene()

        self._log_history = []
        self._refresh_history_display()
        self._search.reset()

        arms_dir  = self._arms_dir(sp)
        arms_path = self._arms_path(sp)
        arms_dir.mkdir(parents=True, exist_ok=True)

        self._status_md.content = f"⏳ Planning **{sp.stem}**…"

        cmd = [
            "arms-plan",
            "--input",      str(sp),
            "--output",     str(arms_path),
            "--output-dir", str(arms_dir) + "/",
        ] + _plan_common_args(float(self._jig_clearance.value))

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                cwd=str(self.step_dir),
            )
            last_line = ""
            for raw in proc.stdout:
                line = raw.rstrip()
                if not line:
                    continue
                last_line = line
                if self._search.feed(line):
                    self._refresh_search_plot()
                    continue            # a trace event, not a message for the log
                self._record_log_line(line)
                nice = _friendly_line(line)
                if nice:
                    self._status_md.content = nice

            proc.wait()
            self._search.finish()
            self._refresh_search_plot(force=True)

            if proc.returncode != 0:
                self._status_md.content = (
                    f"❌ Planning failed (exit {proc.returncode})  \n"
                    f"`{last_line}`"
                )
                return

            self._status_md.content = "✅ Planning complete — loading scene…"
            self._load_scene(arms_path, arms_dir)

        except FileNotFoundError:
            self._status_md.content = (
                "❌ `arms-plan` not found on PATH.  \n"
                "Build the C++ package and ensure its `bin/` is on PATH."
            )
        except Exception as e:
            self._status_md.content = f"❌ Unexpected error: {e}"
        finally:
            self._planning = False

    # ---- scene management ----

    def _clear_scene(self) -> None:
        """Remove all scene geometry and scene-specific GUI elements.

        The background is deliberately re-added: it represents the machine, not
        the plan, so it stays on screen between and before plans.
        """
        self.server.scene.reset()
        self._viewer_state = None
        self._ensure_background()

        # Replace Stage folder contents with placeholder
        for h in self._stage_children:
            try:
                h.remove()
            except Exception:
                pass
        self._stage_children.clear()
        with self._stage_folder:
            self._stage_children = [
                self.server.gui.add_markdown("_No model loaded._")
            ]

        # Replace Parts folder contents with placeholder
        for h in self._parts_children:
            try:
                h.remove()
            except Exception:
                pass
        self._parts_children.clear()
        with self._parts_folder:
            self._parts_children = [
                self.server.gui.add_markdown("_No model loaded._")
            ]

    def _load_scene(self, arms_path: Path, arms_dir: Path) -> None:
        self._clear_scene()

        try:
            doc = SceneDocument.load(str(arms_path))
        except Exception as e:
            self._status_md.content = f"❌ Failed to read {arms_path.name}: {e}"
            return

        print(f"[viewer] Loaded {arms_path}: {len(doc.parts)} parts, "
              f"{len(doc.stages)} stages, {len(doc.grasps)} grasps, {len(doc.jigs)} jigs, "
              f"{len(doc.background)} background meshes")

        state = _ViewerState(doc, self.server, origin=self._scene_origin)
        if self._scene_origin is None:
            # No machine geometry to anchor to — adopt this assembly's centroid
            # and keep it for the rest of the session so the view never shifts
            # again on subsequent loads.
            self._scene_origin = state.origin
        self._viewer_state = state

        state.load_meshes()
        state.load_grasps()
        state.load_jigs(str(arms_dir))
        state.load_jaws()
        state.init_edit_state()
        state.on_edit_changed = lambda: None
        if self._bg_meshes:
            # Config-file background is already on screen at the session origin,
            # which is the same origin this scene uses — nothing to do.
            pass
        else:
            # No config file (e.g. an .arms produced on another machine) — fall
            # back to the copy baked into the file.
            state.load_backgrounds()
            state.show_background = self._show_background
            state._apply_background_visibility()

        self.server.scene.set_up_direction("+z")
        self.server.scene.add_frame(
            "assembly_origin",
            position=(0.0, 0.0, 0.0),
            axes_length=0.03,
            axes_radius=0.001,
        )

        # Inherit current grasp/jig visibility settings
        state.show_unassembled_printed = bool(self._show_unassembled_printed_cb.value)
        state.show_chosen     = bool(self._show_chosen_cb.value)
        state.show_candidates = bool(self._show_candidates_cb.value)
        state.show_failed     = bool(self._show_failed_cb.value)
        state.show_jigs     = bool(self._show_jigs_cb.value)

        # A new scene invalidates the previous edit session.
        self._clear_edit_gui()
        self._edit_mode_cb.value = False
        state.edit_mode = False

        self._build_stage_gui(doc, state)
        self._build_parts_gui(doc, state)

        state.apply_stage(0)
        state._apply_grasp_visibility()
        state._apply_jig_visibility()

        stem = arms_path.stem
        self._status_md.content = (
            f"✅ **{stem}** — {len(doc.parts)} parts, "
            f"{len(doc.stages)} stages, {len(doc.jigs)} jigs"
        )

    # ---- scene-specific GUI ----

    def _build_stage_gui(self, doc: SceneDocument, state: _ViewerState) -> None:
        """Populate the Stage folder with slider + navigation buttons."""
        for h in self._stage_children:
            try:
                h.remove()
            except Exception:
                pass
        self._stage_children.clear()

        n = len(doc.stages)

        with self._stage_folder:
            slider    = self.server.gui.add_slider(
                label="Stage", min=0, max=max(n - 1, 0), step=1, initial_value=0
            )
            lbl       = self.server.gui.add_text("", initial_value=_stage_label(doc, 0))
            btn_prev  = self.server.gui.add_button("◀  Previous step")
            btn_next  = self.server.gui.add_button("▶  Next step")

        self._stage_children = [slider, lbl, btn_prev, btn_next]

        def _apply(idx: int) -> None:
            idx = max(0, min(idx, n - 1))
            state.apply_stage(idx)
            lbl.value = _stage_label(doc, idx)

        @slider.on_update
        def _(_: viser.GuiEvent) -> None:
            _apply(int(slider.value))

        @btn_prev.on_click
        def _(_: viser.GuiEvent) -> None:
            slider.value = max(0, state.current_stage - 1)

        @btn_next.on_click
        def _(_: viser.GuiEvent) -> None:
            slider.value = min(n - 1, state.current_stage + 1)

    def _build_parts_gui(self, doc: SceneDocument, state: _ViewerState) -> None:
        """Populate the Parts folder with per-part visibility checkboxes."""
        for h in self._parts_children:
            try:
                h.remove()
            except Exception:
                pass
        self._parts_children.clear()

        part_cbs: dict[int, object] = {}
        with self._parts_folder:
            for part in doc.parts:
                cb = self.server.gui.add_checkbox(
                    f"{part.name} [{part.type}]", initial_value=True
                )
                part_cbs[part.part_id] = cb

        self._parts_children = list(part_cbs.values())
        state.part_checkboxes = part_cbs

        for part_id, cb in part_cbs.items():
            @cb.on_update
            def _(_: viser.GuiEvent, _pid: int = part_id) -> None:
                handle = state.part_handles.get(_pid)
                if handle is None:
                    return
                if not part_cbs[_pid].value:
                    handle.visible = False
                else:
                    state.apply_stage(state.current_stage)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _stage_label(doc: SceneDocument, idx: int) -> str:
    if not doc.stages or idx >= len(doc.stages):
        return ""
    s  = doc.stages[idx]
    ep = doc.part_by_id(s.edge_part_id) if s.edge_part_id is not None else None
    return (
        f"Step {s.step + 1}/{len(doc.stages)}  "
        f"Place: {ep.name if ep else '—'}  "
        f"({len(s.assembled_part_ids)} assembled)"
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _maximize_when_ready(window) -> None:
    """Maximise the window once the window manager is ready to honour it.

    create_window(maximized=True) is silently dropped by some backends (the Qt
    backend under mutter among them): the maximise request is issued before the
    WM has finished mapping the window, and is lost.  pywebview's own
    @_shown_call gate fires too early to help.  So re-request it until the
    window actually grows, which also copes with a slow-starting WM without
    imposing a fixed delay on a fast one.
    """
    initial_width = window.width
    deadline = time.time() + 10.0

    while time.time() < deadline:
        try:
            window.maximize()
        except Exception:
            pass
        time.sleep(0.4)
        try:
            if int(window.evaluate_js("window.outerWidth")) != initial_width:
                return  # took effect
        except Exception:
            continue  # page not ready yet — keep trying until the deadline


def launch_app(step_dir: str | None = None) -> None:
    """Launch the integrated ARMS Planner in a native window (or browser fallback)."""
    if step_dir is None:
        step_dir = str(_DEFAULT_STEP_DIR if _DEFAULT_STEP_DIR.exists() else Path.cwd())
    try:
        import webview
        _use_webview = True
    except ImportError:
        print(
            "[viewer] pywebview not installed — falling back to browser.\n"
            "[viewer] Install with: pip install pywebview",
            file=sys.stderr,
        )
        _use_webview = False

    server = viser.ViserServer(port=8080)
    app    = ARMSPlannerApp(step_dir, server)  # noqa: F841 — kept alive

    print(f"[viewer] ARMS Planner — watching {Path(step_dir).resolve()}")

    if _use_webview:
        time.sleep(0.4)  # let viser finish binding
        import webview  # re-import after the try block above
        window = webview.create_window(
            "ARMS Planner",
            "http://localhost:8080",
            width=1600,      # restore-down size, once un-maximised
            height=950,
            resizable=True,
            maximized=True,  # honoured by some backends; see _maximize_when_ready
        )
        webview.start(_maximize_when_ready, window)
    else:
        print("[viewer] Open http://localhost:8080 in your browser.  Ctrl-C to quit.")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    launch_app(sys.argv[1] if len(sys.argv) > 1 else None)
