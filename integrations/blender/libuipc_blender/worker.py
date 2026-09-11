# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""External solver entry point. Invoke with a Python containing pyuipc >= 0.0.28."""

import argparse
import json
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np
from materials import cloth_moduli, cloth_stiffness, build_contact_plan
from quality import QualityRecorder
from performance import Timings
from rod import rod_stiffness, validate_linemesh
from affine import pack_affine, cache_vertices

from protocol import (SCHEMA_VERSION, MDDWriter, atomic_json, fingerprint,
                      positive, read_json, validate_mesh, validate_tetmesh, motion_hash, file_sha256, fully_fixed)


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
    if request["schema_version"] not in (2, 3, 4, 5, 6, 7, SCHEMA_VERSION):
        raise ValueError("Unsupported Blender bridge schema")
    settings = request["settings"]
    options = request.get("output_options", {})
    if (not isinstance(options, dict) or set(options) - {"compact_abd"}
            or type(options.get("compact_abd", True)) is not bool):
        raise ValueError("Invalid cache output options")
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
            body = {key: data[key].copy() for key in ("vertices", "triangles", "tetrahedra", "matrix", "pins")}
            if entry["material"]["role"] == "ROD":
                if request["schema_version"] < 7:
                    raise ValueError("Rod jobs require schema 7 or newer")
                body["edges"] = data["edges"].copy()
            if "drive" in entry["material"]:
                body["drive_targets"] = data["drive_targets"].copy()
                if motion_hash(body["drive_targets"]) != entry.get("drive_targets_sha256"):
                    raise ValueError(f"{entry['name']}: corrupted motion samples")
        body.update(name=entry["name"], material=entry["material"])
        if request["schema_version"] >= 5:
            body["id"] = entry["id"]
        bodies.append(body)
    if not bodies or not any(b["material"]["role"] != "STATIC" for b in bodies):
        raise ValueError("Include at least one cloth, rigid body, or volumetric FEM object")
    if request["schema_version"] >= 5:
        from protocol import match_bodies
        match_bodies(request, bodies)
    if fingerprint(settings, bodies, request["schema_version"]) != request["fingerprint"]:
        raise ValueError("Simulation input fingerprint mismatch")
    return request, bodies


def apply_solver_accuracy(config, accuracy, custom=None):
    """Explicit per-scene opt-in; DEFAULT leaves native solver defaults intact."""
    if custom is not None and accuracy != "CUSTOM":
        raise ValueError("Custom solver settings require the CUSTOM profile")
    if accuracy == "CONVERGED":
        config["newton"]["semi_implicit"]["enable"] = 0
        config["newton"]["velocity_tol"] = 0.001
        config["newton"]["velocity_tol_relative"] = 0.0
        config["linear_system"]["tol_rate"] = 1e-6
        config["line_search"]["max_iter"] = 32
    elif accuracy == "CUSTOM":
        from protocol import validate_solver_settings
        values = validate_solver_settings(custom)
        config["linear_system"]["tol_rate"] = values["linear_tolerance"]
        newton = config["newton"]
        newton["velocity_tol"] = values["velocity_tolerance"]
        newton["velocity_tol_relative"] = values["relative_velocity_tolerance"]
        newton["transrate_tol"] = values["transrate_tolerance"]
        newton["semi_implicit"]["enable"] = int(values["semi_implicit"])
        newton["semi_implicit"]["K_min"] = values["k_min"]
        newton["semi_implicit"]["beta_tol"] = values["beta_tolerance"]
        newton["max_iter"] = values["newton_max_iter"]
        newton["min_iter"] = values["newton_min_iter"]
        config["line_search"]["max_iter"] = values["line_search_max_iter"]
    elif accuracy != "DEFAULT":
        raise ValueError(f"Unsupported solver accuracy: {accuracy}")


def apply_cloth_material(mesh, material, shell, bending, moduli_type):
    values = cloth_moduli(material)
    shell.apply_to(mesh, stretch_moduli=moduli_type.youngs_poisson(*values["stretch"]),
                   shear_moduli=moduli_type.youngs_poisson(*values["shear"]),
                   mass_density=material["density"], thickness=material["thickness"],
                   strain_rate=material["strain_rate"])
    if material["role"] == "CLOTH" and values["bending"][0] > 0:
        bending.apply_to(mesh, *values["bending"])


def configure_contacts(scene, settings, bodies):
    plan = build_contact_plan(settings, bodies)
    tabular = scene.contact_tabular()
    tabular.default_model(settings["friction"], settings["resistance"])
    groups = [tabular.default_element()]
    for index, group in enumerate(plan["groups"][1:], 1):
        groups.append(tabular.create(f"blender:{index}:{group['material']}"))
    for model in plan["models"]:
        tabular.insert(groups[model["a"]], groups[model["b"]], model["friction"],
                       model["resistance"], model["enabled"])
    return groups, plan


def simulate(directory, parent):
    timings = Timings()
    with timings.measure("input_load_validate"):
        request, bodies = load_request(directory)
    settings = request["settings"]
    with timings.measure("runtime_import"):
        uipc = load_runtime()
    setup_started = time.perf_counter()
    from uipc import builtin, view
    from uipc.geometry import trimesh, tetmesh, linemesh, label_surface, label_triangle_orient
    from uipc.constitution import (AffineBodyConstitution, DiscreteShellBending,
                                  ElasticModuli2D, StrainLimitingBaraffWitkinShell,
                                  ElasticModuli, StableNeoHookean, SoftTransformConstraint,
                                  HookeanSpring, KirchhoffRodBending)

    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    config = uipc.Scene.default_config()
    config["dt"] = 1.0 / (settings["fps"] * settings["substeps"])
    config["gravity"] = [[float(x)] for x in settings["gravity"]]
    config["contact"]["d_hat"] = settings["d_hat"]
    # Start with standard IPC; inherit the library's semi-implicit/K_min defaults.
    config["contact"]["constitution"] = "ipc"
    config["extras"]["strict_mode"]["enable"] = 1
    accuracy = settings.get("solver_accuracy", "DEFAULT")
    apply_solver_accuracy(config, accuracy, settings.get("solver_settings"))
    scene = uipc.Scene(config)
    contact_groups, contact_plan = configure_contacts(scene, settings, bodies)
    shell = StrainLimitingBaraffWitkinShell()
    bending = DiscreteShellBending()
    abd = AffineBodyConstitution()
    solid = StableNeoHookean()
    spring, rod_bending = HookeanSpring(), KirchhoffRodBending()
    outputs = []
    has_dynamic = False
    frame_count = settings["frame_end"] - settings["frame_start"] + 1
    motion_step = [0]
    for index, body in enumerate(bodies):
        material = body["material"]
        role = material["role"]
        drive = material.get("drive")
        if drive is not None and (role != "RIGID" or material["fixed"]):
            raise ValueError("Motion targets require an unfixed ABD body")
        if role not in ("CLOTH", "RIGID", "STATIC", "FEM", "ROD"):
            raise ValueError(f"{body['name']}: unsupported simulation role")
        for key in ("density", "thickness", "stretch", "shear", "rigidity", "strain_rate"):
            positive(material[key], f"{body['name']}: {key}")
        positive(material["bending"], "bending", allow_zero=True)
        if role == "FEM" and not 0 <= material["poisson"] < 0.5:
            raise ValueError("Poisson ratio must be in [0, 0.5)")
        matrix = np.asarray(body["matrix"], dtype=np.float64)
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1])
                or np.linalg.cond(matrix[:3, :3]) > 1e12):
            raise ValueError(f"{body['name']}: singular or invalid object transform")
        inverse = np.linalg.inv(matrix)
        world_positions = (body["vertices"] @ matrix[:3, :3].T + matrix[:3, 3]) * settings["unit_scale"]
        if role == "ROD":
            rod_stiffness(material)
            world_positions, edges = validate_linemesh(world_positions, body["edges"], body["name"])
        elif role == "FEM":
            world_positions, cells, _boundary, _owners = validate_tetmesh(
                world_positions, body["tetrahedra"], body["name"])
        else:
            world_positions, triangles = validate_mesh(world_positions, body["triangles"], role, body["name"],
                                                       allow_components=drive is not None)
        pins = np.asarray(body["pins"], dtype=np.int32)
        if pins.ndim != 1 or (len(pins) and (pins.min() < 0 or pins.max() >= len(world_positions))):
            raise ValueError(f"{body['name']}: invalid pinned vertex indices")
        center = world_positions.mean(axis=0) if role == "RIGID" else np.zeros(3)
        mesh = (linemesh(world_positions, edges) if role == "ROD" else
                tetmesh(world_positions, cells) if role == "FEM" else trimesh(world_positions - center, triangles))
        label_surface(mesh)
        if role == "FEM":
            label_triangle_orient(mesh)
        if role == "RIGID":
            abd.apply_to(mesh, material["rigidity"], material["density"])
            transform = np.eye(4)
            transform[:3, 3] = center
            view(mesh.transforms())[0] = transform
            view(mesh.instances().find(builtin.is_fixed))[:] = int(material["fixed"])
            has_dynamic |= not material["fixed"]
            thickness = mesh.vertices().find(builtin.thickness)
            if thickness is None:
                thickness = mesh.vertices().create(builtin.thickness, float(material["thickness"]))
            view(thickness)[:] = material["thickness"]
        elif role == "ROD":
            spring.apply_to(mesh, material["rod_stretch"], material["density"], material["thickness"])
            if material["rod_bending"] > 0:
                rod_bending.apply_to(mesh, material["rod_bending"])
        elif role == "FEM":
            positive(material["young_modulus"], "solid Young's modulus")
            solid.apply_to(mesh, ElasticModuli.youngs_poisson(material["young_modulus"], material["poisson"]),
                           mass_density=material["density"])
            view(mesh.vertices().find(builtin.thickness))[:] = material["thickness"]
        else:
            apply_cloth_material(mesh, material, shell, bending, ElasticModuli2D)
        if role != "RIGID":
            fixed = view(mesh.vertices().find(builtin.is_fixed)).reshape(-1)
            if role == "STATIC" or material["fixed"]:
                fixed[:] = 1
            else:
                fixed[pins] = 1
            has_dynamic |= bool(np.any(fixed == 0))
            view(mesh.meta().find(builtin.self_collision))[:] = int(role in ("CLOTH", "FEM", "ROD") and material["self_collision"])
        obj = scene.objects().create(body["name"])
        current, _rest = obj.geometries().create(mesh)
        contact_groups[contact_plan["assignments"][index]].apply_to(current.geometry())
        if drive is not None:
            geometry = current.geometry()
            for key in ("translation_strength", "rotation_strength"):
                positive(drive[key], key, allow_zero=True)
            SoftTransformConstraint().apply_to(geometry, np.array([drive["translation_strength"],drive["rotation_strength"]]))
            view(geometry.instances().find(builtin.is_constrained))[:] = 1
            # Match sample 87's servo-controlled, quasi-static links.
            view(geometry.instances().find(builtin.is_dynamic))[:] = 0
            targets = np.asarray(body["drive_targets"], dtype=np.float64).copy()
            if (targets.shape != ((frame_count-1)*settings["substeps"]+1,4,4)
                    or not np.isfinite(targets).all() or not np.allclose(targets[:,3,:],[0,0,0,1])):
                raise ValueError(f"{body['name']}: invalid target motion shape/values")
            targets[:,:3,3] *= settings["unit_scale"]
            delta = targets @ np.linalg.inv(targets[0])
            rotations = delta[:,:3,:3]
            if (not np.allclose(rotations @ np.transpose(rotations,(0,2,1)), np.eye(3), atol=2e-5)
                    or not np.allclose(np.linalg.det(rotations),1,atol=2e-5)):
                raise ValueError("Robot targets must preserve rigid shape")
            aims = delta @ transform
            def update_target(info, slot=current, values=aims):
                view(slot.geometry().instances().find(builtin.aim_transform))[0] = values[motion_step[0]]
            scene.animator().insert(obj, update_target)
        if role != "STATIC":
            constant = fully_fixed(body)
            compact = (request["schema_version"] >= 8 and role == "RIGID" and not constant
                       and request.get("output_options", {}).get("compact_abd", True))
            rest_map = matrix.copy()
            rest_map[:3,:] *= settings["unit_scale"]
            rest_map[:3,3] -= center
            local_map = inverse.copy()
            local_map[:3,:3] /= settings["unit_scale"]
            outputs.append({"index": index, "slot": current, "role": role,
                            "inverse": inverse, "vertices": len(world_positions), "constant": constant,
                            "encoding": "AFFINE" if compact else "VERTEX",
                            "rest_map": rest_map, "local_map": local_map,
                            "stored_frames": 1 if constant and request["schema_version"] >= 6 else frame_count})

    # Engine must outlive World and all native calls. One process owns one World.
    engine = uipc.Engine("cuda", str(directory / "solver") + os.sep)
    world = uipc.World(engine)
    timings.add("scene_engine_setup", time.perf_counter() - setup_started)
    started = time.monotonic()
    with timings.measure("world_init"):
        world.init(scene)
    if not world.is_valid():
        raise RuntimeError("Scene initialization failed; inspect worker.log for mesh/contact diagnostics")
    with timings.measure("retrieve"):
        world.retrieve()
    writers = []
    statistics = (directory / "solver_steps.jsonl").open("w", encoding="utf-8")
    quality = QualityRecorder(directory, request)
    frame_stats = getattr(engine, "frame_stats", None)
    try:
        for output in outputs:
            with timings.measure("cache_write"):
                writers.append(MDDWriter(directory / f"object_{output['index']:04d}.mdd",
                                         output["stored_frames"], cache_vertices(output), settings["fps"]))
        for frame in range(frame_count):
            if not parent.alive() or (directory / "cancel").exists():
                atomic_json(directory / "status.json", {"state": "cancelled", "frame": frame})
                return
            if frame and has_dynamic:
                for _ in range(settings["substeps"]):
                    if not parent.alive() or (directory / "cancel").exists():
                        atomic_json(directory / "status.json", {"state": "cancelled", "frame": frame})
                        return
                    motion_step[0] += 1
                    with timings.measure("advance"):
                        world.advance()
                    with timings.measure("solver_diagnostics"):
                        if frame_stats is not None:
                            stats = dict(frame_stats())
                            statistics.write(json.dumps({
                                "output_frame": frame + settings["frame_start"], "substep": motion_step[0], **stats,
                            }) + "\n")
                            quality.record_solver(frame + settings["frame_start"], motion_step[0], stats)
                    if not world.is_valid():
                        raise RuntimeError(f"Simulation failed at output frame {frame}")
                with timings.measure("retrieve"):
                    world.retrieve()
                with timings.measure("solver_diagnostics"):
                    statistics.flush()
            for output, writer in zip(outputs, writers):
                if frame and output["constant"]:
                    with timings.measure("motion_diagnostics"):
                        quality.record_stationary(output["index"], frame + settings["frame_start"])
                    if output["stored_frames"] != 1:
                        with timings.measure("cache_write"):
                            writer.append(output["constant_local"])
                    continue
                with timings.measure("output_transform"):
                    geometry = output["slot"].geometry()
                    points = np.asarray(geometry.positions().view()).reshape(-1, 3)
                    if output["role"] == "RIGID":
                        transform = np.asarray(geometry.transforms().view()).reshape(-1, 4, 4)[0]
                        points = points @ transform[:3, :3].T + transform[:3, 3]
                with timings.measure("motion_diagnostics"):
                    quality.record_object(output["index"], frame + settings["frame_start"], points)
                with timings.measure("output_transform"):
                    if output["encoding"] == "AFFINE":
                        local_points = pack_affine(output["local_map"] @ transform @ output["rest_map"])
                    else:
                        points = points / settings["unit_scale"]
                        inverse = output["inverse"]
                        local_points = points @ inverse[:3, :3].T + inverse[:3, 3]
                with timings.measure("cache_write"):
                    writer.append(local_points)
                if output["constant"] and output["stored_frames"] != 1:
                    output["constant_local"] = local_points
            with timings.measure("status_write"):
                atomic_json(directory / "status.json", {
                    "state": "running", "frame": frame + 1, "total": frame_count,
                    "elapsed_seconds": time.monotonic() - started,
                })
        with timings.measure("cache_finalize"):
            for writer in writers:
                writer.close(commit=True)
        with timings.measure("diagnostics_finalize"):
            quality.finish()
            statistics.flush()
            quality_digest = file_sha256(directory / "quality_report.json")
        performance = timings.report()
        performance.update(output_frames=frame_count, native_steps=motion_step[0],
                           output_vertices=sum(o["vertices"] for o in outputs),
                           stored_vertex_samples=sum(cache_vertices(o)*o["stored_frames"] for o in outputs),
                           dense_cache_bytes=sum(8+4*frame_count+12*frame_count*o["vertices"] for o in outputs),
                           cache_bytes=sum(writer.path.stat().st_size for writer in writers),
                           diagnostics_bytes=sum((directory / name).stat().st_size for name in
                               ("solver_steps.jsonl", "quality_frames.jsonl", "quality_report.json")))
        atomic_json(directory / "result.json", {
            "schema_version": request["schema_version"], "fingerprint": request["fingerprint"],
            "build_info": uipc.build_info(), "frames": frame_count,
            "elapsed_seconds": time.monotonic() - started,
            "performance": performance,
            "solver_accuracy": accuracy, "effective_newton": config["newton"],
            "effective_linear_system": config["linear_system"],
            "effective_line_search": config["line_search"],
            "solver_statistics_available": frame_stats is not None,
            "cloth_stiffness": {b["name"]: cloth_stiffness(b["material"])
                                for b in bodies if b["material"]["role"] == "CLOTH"},
            "rod_stiffness": {b["name"]: rod_stiffness(b["material"])
                              for b in bodies if b["material"]["role"] == "ROD"},
            "cache_integrity": 1,
            "effective_contacts": contact_plan,
            "quality_report_sha256": quality_digest,
            "objects": [{"index": o["index"], "vertices": o["vertices"],
                         **({"encoding": o["encoding"]} if request["schema_version"] >= 8 else {}),
                         **({"stored_frames": o["stored_frames"]} if request["schema_version"] >= 6 else {}),
                         "sha256": writer.digest.hexdigest()} for o, writer in zip(outputs, writers)],
        })
        atomic_json(directory / "status.json", {"state": "complete", "frame": frame_count, "total": frame_count})
    finally:
        statistics.close()
        quality.close()
        for writer in writers:
            writer.close()


def prepare_volume(directory, parent):
    """Generate/import once before pin selection; Blender displays actual FEM nodes."""
    request = read_json(directory / "request.json")
    if request["schema_version"] != SCHEMA_VERSION:
        raise ValueError("Unsupported volume preparation schema")
    uipc = load_runtime()
    from uipc.geometry import SimplicialComplexIO, trimesh
    atomic_json(directory / "status.json", {"state": "running", "frame": 0, "total": 1,
                                           "message": "Preparing tetrahedral mesh"})
    if request["operation"] == "import_volume":
        mesh = SimplicialComplexIO().read(str(directory / "input.msh"))
        if mesh.dim() != 3:
            raise ValueError("The selected MSH file does not contain tetrahedral volume cells")
        points = np.array(mesh.positions().view()).reshape(-1, 3)
        cells = np.array(mesh.tetrahedra().topo().view()).reshape(-1, 4)
        report = {"method": "msh_import", "preserve_surface": False}
    else:
        from uipc import geometry
        if not hasattr(geometry, "tetrahedralize"):
            raise RuntimeError("This Python runtime needs the updated libuipc native tetrahedralization API; install the current source build")
        with np.load(directory / "surface.npz", allow_pickle=False) as source:
            original = source["vertices"].copy()
            matrix = source["matrix"].copy()
            triangles = source["triangles"].copy()
        scale = request["unit_scale"]
        world = (original @ matrix[:3, :3].T + matrix[:3, 3]) * scale
        # Native geometry owns validation and the protected-boundary construction.
        mesh, report = geometry.tetrahedralize(trimesh(world, triangles), request["options"])
        points = np.array(mesh.positions().view()).reshape(-1, 3) / scale
        inverse = np.linalg.inv(matrix)
        points = points @ inverse[:3, :3].T + inverse[:3, 3]
        # Blender stores float32 coordinates. Preserve the original values exactly,
        # including under nonuniform scale and translated world-space preparation.
        points[:len(original)] = original
        cells = np.array(mesh.tetrahedra().topo().view()).reshape(-1, 4)
    points = points.astype(np.float32).astype(np.float64)
    points, cells, boundary, _ = validate_tetmesh(points, cells, "Prepared FEM volume")
    if not parent.alive() or (directory / "cancel").exists():
        atomic_json(directory / "status.json", {"state": "cancelled"})
        return
    np.savez(directory / "volume.npz", vertices=points, tetrahedra=cells, triangles=boundary)
    report.update(vertices=len(points), tetrahedra=len(cells), boundary_triangles=len(boundary))
    atomic_json(directory / "result.json", {"schema_version": SCHEMA_VERSION,
        "fingerprint": request["fingerprint"], "report": report, "build_info": uipc.build_info()})
    atomic_json(directory / "status.json", {"state": "complete", "frame": 1, "total": 1})


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
        operation = read_json(directory / "request.json").get("operation", "bake")
        if operation == "import_robot":
            from robot_model import export_robot
            request = read_json(directory / "request.json")
            model = export_robot(request["source"], directory)
            atomic_json(directory / "result.json", {"schema_version": SCHEMA_VERSION,
                "fingerprint": request["fingerprint"], "links": len(model["links"]), "build_info": model["build_info"]})
            atomic_json(directory / "status.json", {"state": "complete", "frame": 1, "total": 1})
        elif operation in ("generate_volume", "import_volume"):
            prepare_volume(directory, parent)
        else:
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
