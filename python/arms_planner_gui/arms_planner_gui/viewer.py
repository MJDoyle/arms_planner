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

MM = 1e-3  # mm → metres

# Default directory for .step input files — arms_planner/input_models/
# Falls back to cwd if the directory doesn't exist (e.g. installed outside the repo).
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
_DEFAULT_STEP_DIR = _REPO_ROOT / "input_models"

# Same file arms-plan reads when baking background meshes into a .arms.  The
# viewer loads it directly so the machine is visible before any plan exists;
# the copy embedded in a .arms is only used as a fallback (see _ensure_background).
_BACKGROUND_MODELS_FILE = _REPO_ROOT / "config" / "background_models.txt"


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
        self.grasp_records:          list              = []
        self.jig_handles:            list              = []
        self.background_handles:     list              = []

        self.current_stage = 0
        self.show_failed     = False
        self.show_candidates = False
        self.show_chosen     = True
        self.show_unassembled_printed = False
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

            if g.chosen and nozzle is not None:
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
            getattr(self, f"grasp_handles_{group}").append(handle)
            self.grasp_records.append((handle, g))

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
        for handle, g in self.grasp_records:
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

            offset_mm = (g.x_mm - part.assembled_pose.x,
                         g.y_mm - part.assembled_pose.y,
                         g.z_mm - part.assembled_pose.z)
            handle.position = self._p(
                base.x + offset_mm[0], base.y + offset_mm[1], base.z + offset_mm[2])

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

        arms_dir  = self._arms_dir(sp)
        arms_path = self._arms_path(sp)
        arms_dir.mkdir(parents=True, exist_ok=True)

        self._status_md.content = f"⏳ Planning **{sp.stem}**…"

        cmd = [
            "arms-plan",
            "--input",      str(sp),
            "--output",     str(arms_path),
            "--output-dir", str(arms_dir) + "/",
        ]

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
                self._record_log_line(line)
                nice = _friendly_line(line)
                if nice:
                    self._status_md.content = nice

            proc.wait()

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
