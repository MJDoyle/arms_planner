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
All scene positions are expressed relative to the assembly centroid so that the
assembly sits at world-origin and viser's default camera looks straight at it.
"""

from __future__ import annotations

import glob
import os
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


# ---------------------------------------------------------------------------
# Colour palettes — viser requires (R, G, B) integers 0-255
# ---------------------------------------------------------------------------

_OPAQUE: dict[str, tuple[int, int, int]] = {
    "external": (64,  153, 242),
    "internal": (217, 140,  51),
    "screw":    (153, 217, 102),
    "unknown":  (178, 178, 178),
}

_TRANSLUCENT: dict[str, tuple[int, int, int]] = {
    "external": (140, 195, 242),
    "internal": (242, 200, 130),
    "screw":    (200, 242, 160),
    "unknown":  (220, 220, 220),
}

_GRASP_COLOURS: dict[str, tuple[int, int, int]] = {
    "accepted":           ( 51, 217,  77),
    "body_collision":     (230,  51,  51),
    "assembly_collision": (242, 153,  26),
    "seal_failed":        (179,  51, 230),
}

_JIG_COLOUR = (204, 128, 26)


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
# Scene state — holds viser handles for a loaded .arms document
# ---------------------------------------------------------------------------

class _ViewerState:
    def __init__(self, doc: SceneDocument, server: viser.ViserServer):
        self.doc      = doc
        self.server   = server
        self.n_stages = len(doc.stages)

        self.part_handles:           dict[int, object] = {}
        self.part_checkboxes:        dict[int, object] = {}
        self.grasp_handles_accepted: list              = []
        self.grasp_handles_rejected: list              = []
        self.jig_handles:            list              = []

        self.current_stage = 0
        self.show_rejected = False
        self.show_jigs     = True

        # Centre the scene at world-origin so viser's default camera works.
        if doc.parts:
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
        for g in self.doc.grasps:
            colour = _GRASP_COLOURS.get(g.status, (128, 128, 128))
            handle = self.server.scene.add_icosphere(
                name=f"grasps/{'ok' if g.status == 'accepted' else 'rej'}/{id(g)}",
                radius=3e-3,
                color=colour,
                position=self._p(g.x_mm, g.y_mm, g.z_mm),
            )
            if g.status == "accepted":
                self.grasp_handles_accepted.append(handle)
            else:
                self.grasp_handles_rejected.append(handle)

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
                    opacity=0.65,
                    side="double",
                    position=jig_offset,
                )
                self.jig_handles.append(handle)
            except Exception as e:
                print(f"[viewer] WARNING: jig failed {stl_path}: {e}", file=sys.stderr)

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
                handle.position = self._p(
                    part.bay_pose.x, part.bay_pose.y, part.bay_pose.z)
                handle.color   = _TRANSLUCENT.get(part.type, _TRANSLUCENT["unknown"])
                handle.opacity = 0.25
                handle.visible = True
            else:
                handle.visible = False

    def _apply_grasp_visibility(self) -> None:
        for h in self.grasp_handles_accepted:
            h.visible = True
        for h in self.grasp_handles_rejected:
            h.visible = self.show_rejected

    def _apply_jig_visibility(self) -> None:
        for h in self.jig_handles:
            h.visible = self.show_jigs


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

        # Scene-specific GUI handles — cleared on each scene load.
        self._stage_children: list = []
        self._parts_children:  list = []

        self._build_gui()

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

        # ---- Stage folder — placeholder replaced on load ----
        with self.server.gui.add_folder("Stage") as self._stage_folder:
            self._stage_children = [
                self.server.gui.add_markdown("_No model loaded._")
            ]

        # ---- Parts folder — placeholder replaced on load ----
        with self.server.gui.add_folder("Parts") as self._parts_folder:
            self._parts_children = [
                self.server.gui.add_markdown("_No model loaded._")
            ]

        # ---- Grasps / Jigs (always visible) ----
        with self.server.gui.add_folder("Grasps"):
            self._show_rejected_cb = self.server.gui.add_checkbox(
                "Show rejected candidates", initial_value=False
            )
        with self.server.gui.add_folder("Jigs"):
            self._show_jigs_cb = self.server.gui.add_checkbox(
                "Show jigs", initial_value=True
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

        @self._show_rejected_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._viewer_state:
                self._viewer_state.show_rejected = bool(self._show_rejected_cb.value)
                self._viewer_state._apply_grasp_visibility()

        @self._show_jigs_cb.on_update
        def _(_: viser.GuiEvent) -> None:
            if self._viewer_state:
                self._viewer_state.show_jigs = bool(self._show_jigs_cb.value)
                self._viewer_state._apply_jig_visibility()

        self._refresh_status()

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
                if line:
                    last_line = line
                    self._status_md.content = f"⏳ `{line}`"

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
        """Remove all scene geometry and scene-specific GUI elements."""
        self.server.scene.reset()
        self._viewer_state = None

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
              f"{len(doc.stages)} stages, {len(doc.grasps)} grasps, {len(doc.jigs)} jigs")

        state = _ViewerState(doc, self.server)
        self._viewer_state = state

        state.load_meshes()
        state.load_grasps()
        state.load_jigs(str(arms_dir))

        self.server.scene.set_up_direction("+z")
        self.server.scene.add_frame(
            "assembly_origin",
            position=(0.0, 0.0, 0.0),
            axes_length=0.03,
            axes_radius=0.001,
        )

        # Inherit current grasp/jig visibility settings
        state.show_rejected = bool(self._show_rejected_cb.value)
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
        window = webview.create_window(   # noqa: F841
            "ARMS Planner",
            "http://localhost:8080",
            width=1600,
            height=950,
            resizable=True,
        )
        webview.start()
    else:
        print("[viewer] Open http://localhost:8080 in your browser.  Ctrl-C to quit.")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    launch_app(sys.argv[1] if len(sys.argv) > 1 else None)
