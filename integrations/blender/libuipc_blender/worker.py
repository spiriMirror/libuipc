# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""External solver entry point. Invoke with a Python containing pyuipc >= 0.0.28."""

import argparse
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from protocol import (SCHEMA_VERSION, MDDWriter, atomic_json, fingerprint,
                      positive, read_json, validate_mesh)


class ParentProcess:
    """A worker exits between substeps if its Blender process has gone away."""

    def __init__(self, pid):
        self.pid = pid
        self.handle = None
        if pid and os.name == "nt":
            import ctypes
            from ctypes import wintypes
            self.kernel = ctypes.WinDLL("kernel32", use_last_error=True)
            self.kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
            self.kernel.OpenProcess.restype = wintypes.HANDLE
            self.kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            self.kernel.WaitForSingleObject.restype = wintypes.DWORD
            self.kernel.CloseHandle.argtypes = [wintypes.HANDLE]
            self.handle = self.kernel.OpenProcess(0x00100000, False, pid)
            if not self.handle:
                raise RuntimeError("Blender parent process is no longer available")

    def alive(self):
        if not self.pid:
            return True
        if self.handle:
            return self.kernel.WaitForSingleObject(self.handle, 0) == 258  # WAIT_TIMEOUT
        return os.getppid() == self.pid

    def close(self):
        if self.handle:
            self.kernel.CloseHandle(self.handle)
            self.handle = None


def load_runtime():
    import uipc
    version = tuple(int(part) for part in uipc.__version__.split("+")[0].split(".")[:3])
    if version < (0, 0, 28):
        raise RuntimeError("Install pyuipc >= 0.0.28 in the external Python environment")
    if not uipc.build_info()["cuda_backend"]:
        raise RuntimeError("This pyuipc build has no CUDA backend")
    return uipc


def load_request(directory):
    request = read_json(directory / "request.json")
    if request["schema_version"] != SCHEMA_VERSION:
        raise ValueError("Unsupported Blender bridge schema")
    settings = request["settings"]
    for key in ("fps", "unit_scale", "d_hat", "resistance"):
        positive(settings[key], key)
    positive(settings["friction"], "friction", allow_zero=True)
    if (not isinstance(settings["substeps"], int) or settings["substeps"] < 1
            or settings["frame_end"] < settings["frame_start"]):
        raise ValueError("Invalid frame range or substeps")
    if len(settings["gravity"]) != 3 or not np.isfinite(settings["gravity"]).all():
        raise ValueError("Invalid gravity")
    bodies = []
    for index, entry in enumerate(request["objects"]):
        # Paths are derived from indices, never from object names or input paths.
        with np.load(directory / f"input_{index:04d}.npz", allow_pickle=False) as data:
            body = {key: data[key].copy() for key in ("vertices", "triangles", "matrix", "pins")}
        body.update(name=entry["name"], material=entry["material"])
        bodies.append(body)
    if not bodies or not any(b["material"]["role"] != "STATIC" for b in bodies):
        raise ValueError("Include at least one cloth or rigid body")
    if fingerprint(settings, bodies) != request["fingerprint"]:
        raise ValueError("Simulation input fingerprint mismatch")
    return request, bodies


def simulate(directory, parent):
    request, bodies = load_request(directory)
    settings = request["settings"]
    uipc = load_runtime()
    from uipc import builtin, view
    from uipc.geometry import trimesh, label_surface
    from uipc.constitution import (AffineBodyConstitution, DiscreteShellBending,
                                  ElasticModuli2D, StrainLimitingBaraffWitkinShell)

    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    config = uipc.Scene.default_config()
    config["dt"] = 1.0 / (settings["fps"] * settings["substeps"])
    config["gravity"] = [[float(x)] for x in settings["gravity"]]
    config["contact"]["d_hat"] = settings["d_hat"]
    # Start with standard IPC; inherit the library's semi-implicit/K_min defaults.
    config["contact"]["constitution"] = "ipc"
    config["extras"]["strict_mode"]["enable"] = 1
    scene = uipc.Scene(config)
    scene.contact_tabular().default_model(settings["friction"], settings["resistance"])
    shell = StrainLimitingBaraffWitkinShell()
    bending = DiscreteShellBending()
    abd = AffineBodyConstitution()
    outputs = []
    frame_count = settings["frame_end"] - settings["frame_start"] + 1
    for index, body in enumerate(bodies):
        material = body["material"]
        role = material["role"]
        if role not in ("CLOTH", "RIGID", "STATIC"):
            raise ValueError(f"{body['name']}: unsupported simulation role")
        for key in ("density", "thickness", "stretch", "shear", "rigidity", "strain_rate"):
            positive(material[key], f"{body['name']}: {key}")
        positive(material["bending"], "bending", allow_zero=True)
        if not 0 <= material["poisson"] < 0.5:
            raise ValueError("Poisson ratio must be in [0, 0.5)")
        matrix = np.asarray(body["matrix"], dtype=np.float64)
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1])
                or np.linalg.cond(matrix[:3, :3]) > 1e12):
            raise ValueError(f"{body['name']}: singular or invalid object transform")
        inverse = np.linalg.inv(matrix)
        world_positions = (body["vertices"] @ matrix[:3, :3].T + matrix[:3, 3]) * settings["unit_scale"]
        world_positions, triangles = validate_mesh(world_positions, body["triangles"], role, body["name"])
        pins = np.asarray(body["pins"], dtype=np.int32)
        if pins.ndim != 1 or (len(pins) and (pins.min() < 0 or pins.max() >= len(world_positions))):
            raise ValueError(f"{body['name']}: invalid pinned vertex indices")
        center = world_positions.mean(axis=0) if role == "RIGID" else np.zeros(3)
        mesh = trimesh(world_positions - center, triangles)
        label_surface(mesh)
        if role == "RIGID":
            abd.apply_to(mesh, material["rigidity"], material["density"])
            transform = np.eye(4)
            transform[:3, 3] = center
            view(mesh.transforms())[0] = transform
            thickness = mesh.vertices().find(builtin.thickness)
            if thickness is None:
                thickness = mesh.vertices().create(builtin.thickness, float(material["thickness"]))
            view(thickness)[:] = material["thickness"]
        else:
            moduli = lambda young: ElasticModuli2D.youngs_poisson(young, material["poisson"])
            shell.apply_to(mesh, stretch_moduli=moduli(material["stretch"]),
                           shear_moduli=moduli(material["shear"]),
                           mass_density=material["density"], thickness=material["thickness"],
                           strain_rate=material["strain_rate"])
            if role == "CLOTH" and material["bending"] > 0:
                bending.apply_to(mesh, material["bending"], material["poisson"])
            fixed = view(mesh.vertices().find(builtin.is_fixed)).reshape(-1)
            if role == "STATIC":
                fixed[:] = 1
            else:
                fixed[pins] = 1
            view(mesh.meta().find(builtin.self_collision))[:] = int(role == "CLOTH" and material["self_collision"])
        obj = scene.objects().create(body["name"])
        current, _rest = obj.geometries().create(mesh)
        if role != "STATIC":
            outputs.append({"index": index, "slot": current, "role": role,
                            "inverse": inverse, "vertices": len(world_positions)})

    # Engine must outlive World and all native calls. One process owns one World.
    engine = uipc.Engine("cuda", str(directory / "solver") + os.sep)
    world = uipc.World(engine)
    started = time.monotonic()
    world.init(scene)
    if not world.is_valid():
        raise RuntimeError("Scene initialization failed; inspect worker.log for mesh/contact diagnostics")
    world.retrieve()
    writers = []
    try:
        for output in outputs:
            writers.append(MDDWriter(directory / f"object_{output['index']:04d}.mdd",
                                     frame_count, output["vertices"], settings["fps"]))
        for frame in range(frame_count):
            if not parent.alive() or (directory / "cancel").exists():
                atomic_json(directory / "status.json", {"state": "cancelled", "frame": frame})
                return
            if frame:
                for _ in range(settings["substeps"]):
                    if not parent.alive() or (directory / "cancel").exists():
                        atomic_json(directory / "status.json", {"state": "cancelled", "frame": frame})
                        return
                    world.advance()
                    if not world.is_valid():
                        raise RuntimeError(f"Simulation failed at output frame {frame}")
                world.retrieve()
            for output, writer in zip(outputs, writers):
                geometry = output["slot"].geometry()
                points = np.asarray(geometry.positions().view()).reshape(-1, 3)
                if output["role"] == "RIGID":
                    transform = np.asarray(geometry.transforms().view()).reshape(-1, 4, 4)[0]
                    points = points @ transform[:3, :3].T + transform[:3, 3]
                points = points / settings["unit_scale"]
                inverse = output["inverse"]
                writer.append(points @ inverse[:3, :3].T + inverse[:3, 3])
            atomic_json(directory / "status.json", {
                "state": "running", "frame": frame + 1, "total": frame_count,
                "elapsed_seconds": time.monotonic() - started,
            })
        for writer in writers:
            writer.close(commit=True)
        atomic_json(directory / "result.json", {
            "schema_version": SCHEMA_VERSION, "fingerprint": request["fingerprint"],
            "build_info": uipc.build_info(), "frames": frame_count,
            "elapsed_seconds": time.monotonic() - started,
            "objects": [{"index": o["index"], "vertices": o["vertices"]} for o in outputs],
        })
        atomic_json(directory / "status.json", {"state": "complete", "frame": frame_count, "total": frame_count})
    finally:
        for writer in writers:
            writer.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", type=Path)
    parser.add_argument("--probe", type=Path)
    parser.add_argument("--parent-pid", type=int)
    args = parser.parse_args()
    if args.probe:
        uipc = load_runtime()
        engine = uipc.Engine("cuda", str(args.probe.parent / "probe_solver") + os.sep)
        atomic_json(args.probe, {"python": sys.executable, "build_info": uipc.build_info(), "cuda": True})
        del engine
        return
    if args.job is None:
        parser.error("--job or --probe is required")
    directory = args.job.resolve()
    parent = None
    try:
        parent = ParentProcess(args.parent_pid)
        simulate(directory, parent)
    except Exception as error:
        traceback.print_exc()
        atomic_json(directory / "status.json", {"state": "error", "message": str(error)})
        raise SystemExit(1) from error
    finally:
        if parent:
            parent.close()


if __name__ == "__main__":
    main()
