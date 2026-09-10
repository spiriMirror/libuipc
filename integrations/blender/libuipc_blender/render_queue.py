# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Owned background-Blender render queues with immutable scene snapshots."""

import os
from pathlib import Path
import time
import uuid

import bpy

from . import bridge
from .protocol import atomic_json, read_json, file_sha256
from .render_protocol import dependency_record, validate_job

_job = None


def shots(scene):
    settings = scene.uipc_settings
    cameras = [row.camera for row in settings.render_cameras if row.enabled]
    if not settings.render_cameras:
        cameras = [scene.camera]
    if not cameras or any(c is None or c.type != "CAMERA" or c.name not in scene.objects for c in cameras):
        raise ValueError("Choose at least one camera linked to this scene")
    if len(set(cameras)) != len(cameras):
        raise ValueError("A camera is listed more than once")
    first, last = settings.render_first, settings.render_last
    if last < first:
        raise ValueError("Render end precedes start")
    bridge.validate_for_render(scene, animation=True)
    if settings.last_bake:
        request = read_json(Path(bpy.path.abspath(settings.last_bake)) / "request.json")
        if first < request["settings"]["frame_start"] or last > request["settings"]["frame_end"]:
            raise ValueError("Render range must stay inside the baked range")
    return [{"camera": c.name, "first": first, "last": last} for c in cameras]


def prepare(scene):
    if not bpy.data.filepath:
        raise ValueError("Save the .blend before creating a render snapshot")
    selected = shots(scene)
    if scene.render.image_settings.file_format != "PNG" or scene.render.use_multiview:
        raise ValueError("The resumable queue requires single-view PNG output")
    if scene.render.use_border:
        raise ValueError("Disable render borders for the resumable full-frame queue")
    if scene.use_nodes and any(n.type == "OUTPUT_FILE" for n in scene.node_tree.nodes):
        raise ValueError("Compositor File Output nodes are not tracked by this queue; use the main PNG output")
    for obj in scene.objects:
        if obj.rigid_body or any(m.type in {"CLOTH", "SOFT_BODY", "FLUID", "PARTICLE_SYSTEM", "DYNAMIC_PAINT"}
                                 for m in obj.modifiers):
            raise ValueError("Bake secondary Blender simulations to a file-backed mesh cache before queueing")
        for modifier in obj.modifiers:
            if modifier.type == "NODES" and modifier.node_group:
                pending, seen = [modifier.node_group], set()
                while pending:
                    tree = pending.pop()
                    if tree in seen:
                        continue
                    seen.add(tree)
                    for node in tree.nodes:
                        if node.bl_idname in {"GeometryNodeSimulationInput", "GeometryNodeSimulationOutput", "GeometryNodeBake"}:
                            raise ValueError("Stateful Geometry Nodes are not snapshot-tracked; use a file-backed cache")
                        if node.type == "GROUP" and node.node_tree:
                            pending.append(node.node_tree)
    for image in bpy.data.images:
        if image.users and image.source in ("SEQUENCE", "MOVIE"):
            raise ValueError("External image sequences/movies are not yet snapshot-tracked")
        if image.users and image.source != "VIEWER" and image.is_dirty and not image.packed_file:
            raise ValueError(f"Save or pack the modified image before queueing: {image.name}")
    paths = set(bpy.utils.blend_paths(absolute=True, packed=True, local=False))
    if scene.uipc_settings.last_bake:
        directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
        paths.update(str(directory / name) for name in ("request.json", "result.json"))
    if os.environ.get("OCIO"):
        paths.add(os.environ["OCIO"])
    dependencies = [dependency_record(p) for p in sorted(paths) if p]
    cycles = None
    if scene.render.engine == "CYCLES" and scene.cycles.device == "GPU":
        preferences = bpy.context.preferences.addons["cycles"].preferences
        preferences.get_devices()
        devices = [d.id for d in preferences.devices if d.use and d.type != "CPU"]
        if not devices:
            raise ValueError("Enable the desired Cycles GPU devices before creating this queue")
        cycles = {"backend": preferences.compute_device_type, "devices": devices}
    configured = scene.uipc_settings.render_directory
    root = Path(bpy.path.abspath(configured)) if configured else Path(bpy.data.filepath).parent / (Path(bpy.data.filepath).stem + "_renders")
    directory = (root / ("render_" + uuid.uuid4().hex)).resolve()
    directory.mkdir(parents=True)
    source = bpy.data.filepath
    # Blender remaps native external-file paths in the saved copy. The live
    # scene, camera, timeline and source filepath remain unchanged.
    if bpy.ops.wm.save_as_mainfile(filepath=str(directory / "scene.blend"), copy=True, relative_remap=True) != {"FINISHED"}:
        raise RuntimeError("Could not save the render snapshot")
    if bpy.data.filepath != source:
        raise RuntimeError("Saving the snapshot unexpectedly changed the active file")
    manifest = {"schema_version": 1, "directory": str(directory), "scene": scene.name,
                "source": source, "snapshot_sha256": file_sha256(directory / "scene.blend"),
                "blender_version": bpy.app.version_string, "dependencies": dependencies, "shots": selected,
                "resolution": [scene.render.resolution_x * scene.render.resolution_percentage // 100,
                               scene.render.resolution_y * scene.render.resolution_percentage // 100],
                "cycles": cycles}
    atomic_json(directory / "render_manifest.json", manifest)
    return directory


def start(scene, resume=False):
    global _job
    from . import runtime
    if runtime.is_running():
        raise RuntimeError("Another simulation/render job is running")
    if resume:
        if not scene.uipc_settings.last_render_job:
            raise ValueError("No previous render queue")
        directory = Path(bpy.path.abspath(scene.uipc_settings.last_render_job)).resolve()
        validate_job(directory)
    else:
        directory = prepare(scene)
    cancel = directory / "render_cancel"
    if cancel.exists():
        cancel.unlink()  # Only this owned job's empty cancellation sentinel.
    atomic_json(directory / "render_status.json", {"state": "starting", "done": 0, "total": 0})
    log = (directory / "render.log").open("ab")
    try:
        process = runtime.launch([bpy.app.binary_path, "--background", "--factory-startup", "--disable-autoexec",
                    "--python-exit-code", "1", "--python", str(Path(__file__).with_name("render_worker.py")),
                    "--", "--job", str(directory), "--parent-pid", str(os.getpid())], log, directory)
    except Exception:
        log.close()
        raise
    _job = {"scene": scene, "directory": directory, "process": process, "log": log, "cancelled_at": None}
    scene.uipc_settings.last_render_job = str(directory)
    scene.uipc_settings.progress = 0.0
    scene.uipc_settings.status = "Preparing render queue"
    return directory


def is_running():
    return _job is not None


def request_cancel():
    if _job is not None and _job["cancelled_at"] is None:
        (_job["directory"] / "render_cancel").touch()
        _job["cancelled_at"] = time.monotonic()
        _job["scene"].uipc_settings.status = "Cancelling render queue"


def poll():
    global _job
    if _job is None:
        return None
    job = _job
    try:
        status = read_json(job["directory"] / "render_status.json")
    except (OSError, ValueError):
        status = {}
    if job["cancelled_at"] is not None and time.monotonic() - job["cancelled_at"] > 3 and job["process"].poll() is None:
        job["process"].kill()
    if job["process"].poll() is None:
        if status.get("total") and job["cancelled_at"] is None:
            job["scene"].uipc_settings.progress = status["done"] / status["total"]
            job["scene"].uipc_settings.status = f"Rendering {status['done']}/{status['total']} PNGs"
        return False
    _job = None
    job["log"].close()
    if job["cancelled_at"] is not None or status.get("state") == "cancelled":
        job["scene"].uipc_settings.status = "Render cancelled; completed frames retained for resume"
        return {"cancelled": True}
    if job["process"].returncode != 0 or status.get("state") != "complete":
        raise RuntimeError(f"{status.get('message', 'Render worker failed')}. Log: {job['directory'] / 'render.log'}")
    job["scene"].uipc_settings.progress = 1
    job["scene"].uipc_settings.status = f"Rendered {status['rendered']}; verified/skipped {status['skipped']} PNGs"
    return status


def stop():
    global _job
    if _job is not None:
        job, _job = _job, None
        if job["process"].poll() is None:
            job["process"].kill()
            job["process"].wait(timeout=5)
        job["log"].close()


def run_blocking(scene, resume=False, timeout=86400):
    start(scene, resume)
    started = time.monotonic()
    try:
        while True:
            result = poll()
            if result is not False:
                return result
            if time.monotonic() - started > timeout:
                raise TimeoutError("Render queue timed out; completed frames can be resumed")
            time.sleep(.1)
    except Exception:
        stop()
        raise
