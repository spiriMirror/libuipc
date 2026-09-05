# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""A single owned subprocess, polled on Blender's main thread (no Python threads)."""

import os
from pathlib import Path
import shutil
import subprocess
import time

import bpy

from .bridge import attach_cache, export_job, export_volume_job, attach_volume, export_robot_job
from .protocol import atomic_json, read_json

_job = None


def python_command(configured):
    if not configured:
        addon = bpy.context.preferences.addons.get(__package__)
        if addon:
            configured = addon.preferences.python_executable
    if configured:
        path = Path(bpy.path.abspath(configured)).expanduser()
        if not path.is_file():
            raise ValueError(f"External Python executable not found: {path}")
        return [str(path)]
    candidates = ("python", "python3") if os.name == "nt" else ("python3", "python")
    for name in candidates:
        path = shutil.which(name)
        if path and "WindowsApps" not in path and Path(path).resolve() != Path(bpy.app.binary_path).resolve():
            return [path]
    raise ValueError("Set External Python in the libuipc panel to a Python with pyuipc installed")


def launch(command, log, cwd):
    environment = os.environ.copy()
    # Blender's Python setup must not leak into a different interpreter.
    for name in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV"):
        environment.pop(name, None)
    environment["PYTHONUNBUFFERED"] = "1"
    kwargs = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {}
    return subprocess.Popen(command, cwd=cwd, env=environment, stdin=subprocess.DEVNULL,
                            stdout=log, stderr=subprocess.STDOUT, **kwargs)


def is_running():
    return _job is not None


def start(scene, volume_object=None, volume_file=None, robot_file=None):
    global _job
    if _job is not None:
        raise RuntimeError("A libuipc job is already running")
    command = python_command(scene.uipc_settings.python_executable)
    if robot_file is not None:
        directory, request = export_robot_job(scene, robot_file)
    elif volume_object is not None or volume_file is not None:
        directory, request = export_volume_job(scene, volume_object, volume_file)
    else:
        directory, request = export_job(scene)
    log = (directory / "worker.log").open("wb")
    try:
        process = launch(command + [str(Path(__file__).with_name("worker.py")), "--job", str(directory),
                                    "--parent-pid", str(os.getpid())], log, directory)
    except Exception:
        log.close()
        raise
    _job = {"scene": scene, "process": process, "log": log, "directory": directory,
            "request": request, "cancelled_at": None, "object": volume_object}
    scene.uipc_settings.status = "Preparing robot" if robot_file else "Preparing tetrahedral mesh" if request.get("operation") else "Initializing CUDA simulation"
    scene.uipc_settings.progress = 0.0
    return directory


def request_cancel():
    if _job is not None and _job["cancelled_at"] is None:
        (_job["directory"] / "cancel").touch()
        _job["cancelled_at"] = time.monotonic()
        _job["scene"].uipc_settings.status = "Cancelling simulation"


def poll():
    """Return result on success, False while running, None if idle; raise on failure."""
    global _job
    if _job is None:
        return None
    job = _job
    process, scene = job["process"], job["scene"]
    status_path = job["directory"] / "status.json"
    try:
        state = read_json(status_path) if status_path.exists() else {}
    except (OSError, ValueError):
        state = {}  # Atomic replacement may transiently conflict with an OS reader.
    if state.get("state") == "running" and job["cancelled_at"] is None:
        scene.uipc_settings.progress = state["frame"] / state["total"]
        scene.uipc_settings.status = state.get("message", f"Baking {state['frame']}/{state['total']} frames")
    if job["cancelled_at"] is not None and time.monotonic() - job["cancelled_at"] > 3 and process.poll() is None:
        process.kill()
    if process.poll() is None:
        return False
    job["log"].close()
    _job = None
    if job["cancelled_at"] is not None or state.get("state") == "cancelled":
        scene.uipc_settings.status = "Cancelled; previous bake retained"
        scene.uipc_settings.progress = 0.0
        return {"cancelled": True}
    if process.returncode != 0 or state.get("state") != "complete":
        message = state.get("message", f"Worker exited with code {process.returncode}")
        raise RuntimeError(f"{message}. Log: {job['directory'] / 'worker.log'}")
    if job["request"].get("operation") == "import_robot":
        from .robot_ui import attach_robot
        result = attach_robot(scene, job["directory"], job["request"])
        scene.uipc_settings.progress = 1.0
        return result
    if job["request"].get("operation") in ("generate_volume", "import_volume"):
        result = attach_volume(scene, job["object"], job["directory"], job["request"])
        scene.uipc_settings.progress = 1.0
        return result
    result = attach_cache(scene, job["directory"], job["request"])
    scene.uipc_settings.progress = 1.0
    scene.uipc_settings.status = f"Baked {result['frames']} frames with pyuipc {result['build_info']['version']}"
    return result


def stop():
    """Only terminate our child; called before file loading or addon unregister."""
    global _job
    if _job is not None:
        job, _job = _job, None
        if job["process"].poll() is None:
            job["process"].kill()
            job["process"].wait(timeout=5)
        job["log"].close()
        atomic_json(job["directory"] / "status.json", {"state": "cancelled"})


def bake_blocking(scene, timeout=3600, volume_object=None, volume_file=None, robot_file=None):
    start(scene, volume_object, volume_file, robot_file)
    started = time.monotonic()
    try:
        while True:
            result = poll()
            if result is not False:
                return result
            if time.monotonic() - started > timeout:
                raise TimeoutError("libuipc bake timed out")
            time.sleep(0.1)
    except Exception:
        stop()
        raise
