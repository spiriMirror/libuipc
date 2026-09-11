# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Controlled CUDA replay of exported inputs; never overwrite a Blender bake.

Run with the external pyuipc Python, not Blender's Python. Diagnostic caches
have distinct fingerprints and are not substitutes for the authored scene's
cache. The full World, rest meshes, materials and timestep are retained.
"""

import argparse
import copy
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
import worker
from protocol import atomic_json, fingerprint, motion_hash, positive, read_json


def prepare(args):
    request, bodies = worker.load_request(args.source.resolve())
    request, bodies = copy.deepcopy(request), copy.deepcopy(bodies)
    settings = request["settings"]
    last = settings["frame_end"] if args.end_frame is None else args.end_frame
    if not settings["frame_start"] <= last <= settings["frame_end"]:
        raise ValueError("End frame must be within the source bake's frame range")
    if args.linear_tol is not None:
        positive(args.linear_tol, "linear tolerance")
    if not np.isfinite(args.robot_offset).all():
        raise ValueError("Robot offset must be finite")
    settings["frame_end"] = last
    settings.pop("solver_settings", None)
    if args.profile == "DEFAULT":
        settings.pop("solver_accuracy", None)
    else:
        settings["solver_accuracy"] = args.profile
    settings["diagnostic_replay"] = {
        "source_fingerprint": request["fingerprint"],
        "hold": args.hold,
        "robot_offset_m": args.robot_offset,
        "linear_tol": args.linear_tol,
        "semi_implicit": args.semi,
    }
    count = (last - settings["frame_start"]) * settings["substeps"] + 1
    # A fresh directory is mandatory. Original inputs/caches stay untouched.
    args.output.mkdir(parents=True, exist_ok=False)
    offset = np.asarray(args.robot_offset) / settings["unit_scale"]
    for index, body in enumerate(bodies):
        arrays = {
            key: body[key]
            for key in ("vertices", "triangles", "tetrahedra", "matrix", "pins")
        }
        if "drive_targets" in body:
            targets = body["drive_targets"][:count].copy()
            if args.hold:
                targets[:] = targets[0]
            arrays["matrix"][:3, 3] += offset
            targets[:, :3, 3] += offset
            arrays["drive_targets"] = targets
            digest = motion_hash(targets)
            # Hash actual replayed samples, not an obsolete authored F-curve.
            body["material"]["drive"]["signature"] = "diagnostic:" + digest
            request["objects"][index]["drive_targets_sha256"] = digest
        request["objects"][index]["material"] = body["material"]
        np.savez(args.output / f"input_{index:04d}.npz", **arrays)
    request["fingerprint"] = fingerprint(settings, bodies)
    atomic_json(args.output / "request.json", request)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--end-frame", type=int)
    parser.add_argument(
        "--profile", choices=("DEFAULT", "CONVERGED"), default="DEFAULT"
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="Hold all driven bodies at their initial targets",
    )
    parser.add_argument(
        "--robot-offset",
        type=float,
        nargs=3,
        default=[0, 0, 0],
        metavar=("X", "Y", "Z"),
        help="Translate driven bodies and targets by this offset in meters",
    )
    parser.add_argument(
        "--linear-tol", type=float, help="Override only PCG tol_rate for an ablation"
    )
    parser.add_argument(
        "--semi",
        type=int,
        choices=(0, 1),
        help="Override only semi-implicit enable for an ablation",
    )
    parser.add_argument(
        "--prepare-only",
        action="store_true",
        help="Validate/write inputs without invoking CUDA",
    )
    parser.add_argument("--worker-process", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if not args.worker_process:
        prepare(args)
        if args.prepare_only:
            worker.load_request(args.output)
            print("REPLAY_INPUTS_VERIFIED", args.output)
            return
        # Native strict failures can terminate the interpreter without raising
        # a Python exception. An owning parent records that exit as failure.
        with (args.output / "worker.log").open("x", encoding="utf-8") as log:
            process = subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    *sys.argv[1:],
                    "--worker-process",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )
        status = args.output / "status.json"
        if process.returncode:
            last = read_json(status) if status.exists() else None
            atomic_json(
                status,
                {
                    "state": "failed",
                    "exit_code": process.returncode,
                    "last_progress": last,
                },
            )
            raise SystemExit(process.returncode)
        print("REPLAY_" + read_json(status)["state"].upper(), args.output)
        return
    original = worker.apply_solver_accuracy

    def accuracy(config, profile, custom=None):
        original(config, profile, custom)
        if args.linear_tol is not None:
            config["linear_system"]["tol_rate"] = args.linear_tol
        if args.semi is not None:
            config["newton"]["semi_implicit"]["enable"] = args.semi

    worker.apply_solver_accuracy = accuracy
    parent = worker.ParentProcess(os.getppid())
    try:
        worker.simulate(args.output, parent)
        print(
            "REPLAY_" + read_json(args.output / "status.json")["state"].upper(),
            args.output,
        )
    except Exception as error:
        atomic_json(
            args.output / "status.json", {"state": "failed", "error": str(error)}
        )
        raise
    finally:
        worker.apply_solver_accuracy = original
        parent.close()


if __name__ == "__main__":
    main()
