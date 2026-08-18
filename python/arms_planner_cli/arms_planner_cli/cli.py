"""arms CLI — thin wrapper around the arms-plan binary and the viser viewer."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys


# ---------------------------------------------------------------------------
# Subcommand: plan
# ---------------------------------------------------------------------------

def cmd_plan(args: argparse.Namespace) -> None:
    binary = shutil.which("arms-plan")
    if binary is None:
        sys.exit(
            "ERROR: arms-plan binary not found on PATH.\n"
            "Build the C++ package and add its install/bin/ to PATH."
        )

    cmd = [binary, "--input", args.input]

    if args.output:
        cmd += ["--output", args.output]
    if args.output_dir:
        cmd += ["--output-dir", args.output_dir]
    if args.no_grasps:
        cmd.append("--no-grasps")
    if args.no_jigs:
        cmd.append("--no-jigs")
    if args.no_path:
        cmd.append("--no-path")

    proc = subprocess.run(cmd, check=False)
    sys.exit(proc.returncode)


# ---------------------------------------------------------------------------
# Subcommand: view
# ---------------------------------------------------------------------------

def cmd_view(args: argparse.Namespace) -> None:
    try:
        from arms_planner_gui.viewer import launch_app
    except ImportError as e:
        sys.exit(
            f"ERROR: arms_planner_gui is not installed ({e}).\n"
            "Run: pip install arms_planner/python/arms_planner_gui"
        )
    launch_app(args.step_dir)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        prog="arms",
        description="ARMS fabrication planner",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # ---- plan ----
    p_plan = sub.add_parser("plan", help="Run the assembly planner on a STEP file")
    p_plan.add_argument(
        "--input", required=True, metavar="FILE", help="STEP input file"
    )
    p_plan.add_argument(
        "--output", metavar="FILE", help="Output .arms file (default: <stem>.arms)"
    )
    p_plan.add_argument(
        "--output-dir",
        metavar="DIR",
        help="Pipeline output directory (default: arms_output/<stem>/)",
    )
    p_plan.add_argument("--no-grasps", action="store_true", help="Skip grasp generation")
    p_plan.add_argument("--no-jigs", action="store_true", help="Skip jig STL generation")
    p_plan.add_argument("--no-path", action="store_true", help="Skip path planning")
    p_plan.set_defaults(func=cmd_plan)

    # ---- view ----
    p_view = sub.add_parser(
        "view",
        help="Launch the integrated planner + viewer GUI",
    )
    p_view.add_argument(
        "step_dir",
        metavar="DIR",
        nargs="?",
        default=None,
        help="Directory containing .step files (default: arms_planner/input_models/)",
    )
    p_view.set_defaults(func=cmd_view)

    args = parser.parse_args()
    args.func(args)
