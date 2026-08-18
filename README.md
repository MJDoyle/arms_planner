# ARMS Planner

Automated assembly sequence planner for robotic assembly tasks. Given a STEP file of a multi-part assembly, it computes the order in which parts must be assembled, generates vacuum grasp candidates for each step, and produces 3D-printable jig fixtures to hold sub-assemblies in place. Results are viewed in an integrated 3D GUI.

```
input_models/my_assembly.step
        │
        ▼
   arms-plan (C++)
        │
        ├── my_assembly.arms  ──► arms view (3D browser)
        │     ├── manifest.json         stage-by-stage plan,
        │     └── part_N.glb (×N)      poses, grasp candidates
        │
        ├── jig_*.stl  ──► PrusaSlicer / FDM printer
        └── assembly_plan.yaml
```

---

## Prerequisites

### System packages (Ubuntu 24.04)

```bash
sudo apt install \
  cmake build-essential \
  libocct-foundation-dev libocct-modeling-data-dev \
  libocct-modeling-algorithms-dev libocct-data-exchange-dev \
  libocct-visualization-dev libocct-ocaf-dev \
  libyaml-cpp-dev libboost-all-dev libminizip-dev \
  python3.12 python3.12-venv \
  libxcb-cursor0      # required by Qt 6.5+ for the native GUI window
```

### Coal (collision detection) — build from source

Coal is not available via apt and must be installed from source before building arms-plan.

```bash
git clone https://github.com/coal-library/coal.git
cd coal
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../..
```

---

## 1 — Build the C++ planner

```bash
git clone https://github.com/<your-org>/arms_planner.git
cd arms_planner

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..
```

Add the binary to your PATH (add this line to `~/.bashrc` to make it permanent):

```bash
export PATH="$HOME/Development/arms_planner/build:$PATH"
```

Verify:

```bash
arms-plan --help
```

---

## 2 — Set up the Python environment

```bash
python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip setuptools

# Core data model (SceneDocument, etc.)
pip install -e python/arms_planner_core

# 3D viewer (viser + pywebview)
pip install -e python/arms_planner_gui

# CLI entry point (`arms` command)
pip install -e python/arms_planner_cli

# Native window backend for pywebview on Linux
pip install pyqt6 PyQt6-WebEngine qtpy
```

Verify:

```bash
arms --help
```

---

## Usage

### Interactive GUI

```bash
arms view                        # watches arms_planner/input_models/ by default
arms view /path/to/step/files    # or point at a specific directory
```

The window opens automatically. Place `.step` files in `input_models/`, select one from the dropdown, then:

- **Start Plan** — runs the full planning pipeline and loads the result when done
- **Show Plan** — loads an existing `.arms` file without re-planning

Use the Stage slider or **◀ Previous / Next ▶** buttons to step through the assembly sequence. Assembled parts are shown opaque; parts waiting in the bay are shown translucent.

### Command-line (headless)

```bash
arms plan --input input_models/my_assembly.step
```

Output goes to `input_models/arms_output/my_assembly/`. Pass `--help` for all options including `--no-grasps`, `--no-jigs`, `--no-path`.

---

## Repository layout

```
arms_planner/
├── CMakeLists.txt              C++ build (arms-plan binary)
├── src/                        C++ source — planner, grasp gen, jig gen
├── include/assembler/          C++ headers
├── config/
│   └── arms_prusa_config.ini   PrusaSlicer profile for jig printing
├── input_models/               Drop .step files here; output goes in arms_output/
├── python/
│   ├── arms_planner_core/      Data model — SceneDocument, PartRecord, etc.
│   ├── arms_planner_gui/       Viser-based 3D viewer + integrated GUI
│   └── arms_planner_cli/       `arms` CLI entry point
└── build/                      CMake build output (arms-plan binary lives here)
```

---

## Configuration

### PrusaSlicer (optional — for jig G-code)

The planner looks for `config/arms_prusa_config.ini` at build time. If PrusaSlicer is not installed, the slicing step is silently skipped and all other outputs are still generated.

To use a different config file, override the CMake variable at configure time:

```bash
cmake .. -DARMS_PLANNER_CONFIG_DIR=/path/to/your/config
```

### Overriding the binary path

The `arms plan` subcommand shells out to `arms-plan`. If you have multiple builds, set `arms_plan_binary` in the ROS node parameters, or ensure the right build directory is first on your PATH.

---

## Dependencies summary

| Dependency | Version tested | Source |
|---|---|---|
| OpenCASCADE (OCCT) | 7.6.3 | apt |
| Coal | 3.0.3 | build from source |
| yaml-cpp | 0.8.0 | apt |
| Boost (json) | 1.83 | apt |
| minizip | 1.3 | apt |
| Python | 3.12 | apt |
| viser | 0.2.23 | pip |
| pywebview | 6.x | pip |
| PyQt6 + PyQt6-WebEngine | 6.x | pip |
