# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Render a verified saved snapshot in background Blender, never run simulation."""

import argparse
from pathlib import Path
import sys
import time
import traceback

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from protocol import atomic_json, file_sha256
from render_protocol import validate_job, frame_paths, completed_frame, png_info, replace_with_retry, dependency_stamps
from worker import ParentProcess


def render(directory, parent):
    job, signature = validate_job(directory, verify_files=False)
    baseline = dependency_stamps(directory, job)
    validate_job(directory)
    def unchanged_inputs():
        if dependency_stamps(directory, job) != baseline:
            raise ValueError("Render inputs changed during execution; restore them or create a new queue")
    unchanged_inputs()
    if bpy.app.version_string != job["blender_version"]:
        raise ValueError("Render job belongs to a different Blender version")
    bpy.ops.wm.open_mainfile(filepath=str(directory / "scene.blend"))
    if bpy.app.autoexec_fail:
        raise ValueError("Snapshot requires disabled Python auto-execution")
    scene = bpy.data.scenes[job["scene"]]
    bpy.context.window.scene = scene
    if job.get("cycles"):
        preferences = bpy.context.preferences.addons["cycles"].preferences
        preferences.compute_device_type = job["cycles"]["backend"]
        preferences.get_devices()
        found = set()
        for device in preferences.devices:
            device.use = device.id in job["cycles"]["devices"]
            if device.use:
                found.add(device.id)
        if found != set(job["cycles"]["devices"]):
            raise ValueError("The queued Cycles devices are not available")
    total = sum(shot["last"] - shot["first"] + 1 for shot in job["shots"])
    done = rendered = skipped = 0
    started = time.monotonic()
    for camera_index, shot in enumerate(job["shots"]):
        camera = scene.objects.get(shot["camera"])
        if camera is None or camera.type != "CAMERA":
            raise ValueError("Snapshot camera is missing")
        for frame in range(shot["first"], shot["last"] + 1):
            if not parent.alive() or (directory / "render_cancel").exists():
                atomic_json(directory / "render_status.json", {"state": "cancelled", "done": done, "total": total})
                return
            unchanged_inputs()
            if completed_frame(directory, camera_index, frame, signature, job["resolution"]):
                skipped += 1
            else:
                scene.frame_set(frame)
                if bpy.app.autoexec_fail:
                    raise ValueError("Frame requires disabled Python auto-execution")
                scene.camera = camera  # An explicit shot wins over timeline camera markers.
                final, receipt = frame_paths(directory, camera_index, frame)
                final.parent.mkdir(exist_ok=True)
                temporary = final.with_suffix(".partial.png")
                scene.render.filepath = str(temporary)
                if bpy.ops.render.render(write_still=True) != {"FINISHED"}:
                    raise RuntimeError(f"Render was cancelled at camera {camera_index}, frame {frame}")
                if png_info(temporary) != tuple(job["resolution"]):
                    raise ValueError("Rendered PNG dimensions differ from the snapshot")
                unchanged_inputs()
                replace_with_retry(temporary, final)
                atomic_json(receipt, {"job_signature": signature, "camera_index": camera_index,
                                      "frame": frame, "sha256": file_sha256(final), "input_guard": True})
                rendered += 1
            done += 1
            atomic_json(directory / "render_status.json", {"state": "rendering", "done": done, "total": total,
                        "camera": shot["camera"], "frame": frame, "rendered": rendered, "skipped": skipped,
                        "elapsed_seconds": time.monotonic() - started})
    # Catch external changes during a running job, not only on resume.
    validate_job(directory)
    atomic_json(directory / "render_status.json", {"state": "complete", "done": done, "total": total,
                "rendered": rendered, "skipped": skipped, "elapsed_seconds": time.monotonic() - started})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", type=Path, required=True)
    parser.add_argument("--parent-pid", type=int, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    directory = args.job.resolve()
    parent = ParentProcess(args.parent_pid)
    try:
        render(directory, parent)
    except Exception as error:
        traceback.print_exc()
        atomic_json(directory / "render_status.json", {"state": "failed", "message": str(error)})
        raise
    finally:
        parent.close()


if __name__ == "__main__":
    main()
