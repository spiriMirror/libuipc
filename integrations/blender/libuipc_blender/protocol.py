# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Versioned file protocol shared by Blender and the external Python worker.

No bpy or uipc imports: this module runs in both Python interpreters.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import struct
import time

import numpy as np

SCHEMA_VERSION = 8
MODIFIER_NAME = "libuipc Cache"
OBJECT_FIELDS = (
    "role", "density", "thickness", "stretch", "shear", "bending",
    "poisson", "strain_rate", "rigidity", "young_modulus", "fixed",
    "self_collision", "pin_group", "pin_threshold",
)

SOLVER_FIELDS = (
    "linear_tolerance", "velocity_tolerance", "relative_velocity_tolerance",
    "transrate_tolerance", "semi_implicit", "k_min", "beta_tolerance",
    "newton_max_iter", "newton_min_iter", "line_search_max_iter",
)


def validate_solver_settings(values):
    """Normalize scientific-notation UI text before hashing or native calls."""
    if not isinstance(values, dict) or set(values) != set(SOLVER_FIELDS):
        raise ValueError("Custom solver settings require: " + ", ".join(SOLVER_FIELDS))
    result = dict(values)
    for name in ("linear_tolerance", "velocity_tolerance", "relative_velocity_tolerance",
                 "transrate_tolerance", "beta_tolerance"):
        try:
            if isinstance(values[name], bool):
                raise ValueError()
            value = float(values[name])
        except (TypeError, ValueError):
            raise ValueError(f"{name}: enter a number, e.g. 1e-6") from None
        positive(value, name, allow_zero=name in ("relative_velocity_tolerance", "beta_tolerance"))
        result[name] = value
    if result["linear_tolerance"] >= 1:
        raise ValueError("linear_tolerance must be in (0, 1)")
    if result["beta_tolerance"] > 1:
        raise ValueError("beta_tolerance must be in [0, 1]")
    if type(values["semi_implicit"]) is not bool:
        raise ValueError("semi_implicit must be a boolean")
    for name, low, high in (("k_min", 0, 100000), ("newton_min_iter", 0, 100000),
                            ("newton_max_iter", 1, 100000), ("line_search_max_iter", 1, 128)):
        if type(values[name]) is not int or not low <= values[name] <= high:
            raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    if values["newton_min_iter"] > values["newton_max_iter"]:
        raise ValueError("Newton minimum iterations cannot exceed maximum iterations")
    return result


def motion_hash(array):
    return hashlib.sha256(np.ascontiguousarray(array, dtype="<f8").tobytes()).hexdigest()


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    # Windows readers and antivirus can briefly deny FILE_SHARE_DELETE. Never
    # truncate the live status file; retry atomic replacement after readers close.
    for attempt in range(100):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 99:
                raise
            time.sleep(0.01)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def fingerprint(settings, bodies, schema_version=SCHEMA_VERSION):
    """Include vertex values/order, topology, pins, transforms, and all physics inputs."""
    digest = hashlib.sha256()
    digest.update(json.dumps(settings, sort_keys=True, allow_nan=False).encode())
    if schema_version >= 5:
        bodies = sorted(bodies, key=lambda b: b.get("id", b["name"]))
    for body in bodies:
        material = dict(body["material"])
        if "drive" in material:
            drive = dict(material["drive"])
            if schema_version >= 5 and "target_id" in drive:
                drive["target"] = drive["target_id"]
                drive["signature"] = drive["stable_signature"]
            drive.pop("target_id", None)
            drive.pop("stable_signature", None)
            material["drive"] = drive
        digest.update(json.dumps(material, sort_keys=True, allow_nan=False).encode())
        identifier = body.get("id", body["name"]) if schema_version >= 5 else body["name"]
        digest.update(identifier.encode("utf-8"))
        for key, dtype in (("vertices", "<f8"), ("triangles", "<i4"),
                           ("tetrahedra", "<i4"), ("matrix", "<f8"),
                           ("pins", "<i4")):
            if schema_version == 1 and key == "tetrahedra":
                continue
            values = body.get(key, np.empty((0, 4), dtype=np.int32)) if key == "tetrahedra" else body[key]
            data = np.ascontiguousarray(values, dtype=dtype)
            digest.update(str(data.shape).encode())
            digest.update(data.tobytes())
        if body["material"].get("role") == "ROD" and schema_version >= 7:
            data = np.ascontiguousarray(body["edges"], dtype="<i4")
            digest.update(str(data.shape).encode())
            digest.update(data.tobytes())
    return digest.hexdigest()


def cache_fingerprint(request, settings, bodies):
    """Keep v0.1 cloth/ABD bakes valid when the new fixed/FEM features are unused."""
    schema = request.get("schema_version", 1)
    if schema < 4:
        legacy_bodies = []
        for body in bodies:
            material = dict(body["material"])
            for name in ("stretch_poisson", "shear_poisson", "bending_poisson"):
                if name in material:
                    if material[name] != material.get("poisson"):
                        return None
                    material.pop(name)
            legacy_bodies.append({**body, "material": material})
        bodies = legacy_bodies
    if schema == 1:
        legacy = []
        if len(bodies) != len(request["objects"]):
            return None
        for body, old in zip(bodies, request["objects"]):
            if body["material"].get("fixed", False) or body["material"]["role"] == "FEM" or "drive" in body["material"]:
                return None
            fields = old["material"].keys()
            legacy.append({**body, "material": {key: body["material"][key] for key in fields}})
        return fingerprint(settings, legacy, schema_version=1)
    if schema not in (2, 3, 4, 5, 6, 7, SCHEMA_VERSION):
        return None
    return fingerprint(settings, bodies, schema_version=schema)


def match_bodies(request, bodies):
    """Return current bodies in immutable request order, not current name order."""
    field = "id" if request["schema_version"] >= 5 else "name"
    expected = [o.get(field) for o in request["objects"]]
    actual = [b.get(field) for b in bodies]
    if (any(not isinstance(k, str) or not k for k in expected + actual)
            or len(set(expected)) != len(expected) or len(set(actual)) != len(actual)
            or set(expected) != set(actual)):
        raise ValueError(f"Cached object {field}s are missing, duplicated or changed; rebake after fixing identities")
    lookup = dict(zip(actual, bodies))
    return [lookup[key] for key in expected]


def positive(value, name, allow_zero=False):
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'non-negative' if allow_zero else 'positive'}")


def fully_fixed(body):
    """Only proven fixed inputs qualify, never merely small observed motion."""
    material = body["material"]
    if "drive" in material:
        return False
    if material.get("fixed", False) or material["role"] == "STATIC":
        return True
    if material["role"] in ("CLOTH", "FEM", "ROD"):
        pins = np.asarray(body["pins"])
        count = len(body["vertices"])
        return bool(count and len(pins) and pins.min() >= 0 and pins.max() < count
                    and len(np.unique(pins)) == count)
    return False


def validate_result(request, result, vertex_counts, fixed_flags=None):
    """Never trust result-provided counts, indices or provenance in isolation."""
    frames = request["settings"]["frame_end"] - request["settings"]["frame_start"] + 1
    if (result.get("schema_version") != request["schema_version"]
            or result.get("fingerprint") != request["fingerprint"]):
        raise ValueError("Bake result provenance does not match the request")
    if type(result.get("frames")) is not int or result["frames"] != frames:
        raise ValueError("Bake result has the wrong frame count")
    outputs = result.get("objects")
    expected = [i for i, entry in enumerate(request["objects"]) if entry["material"]["role"] != "STATIC"]
    if not isinstance(outputs, list) or any(not isinstance(o, dict) for o in outputs):
        raise ValueError("Bake result object list is invalid")
    if [o.get("index") for o in outputs] != expected:
        raise ValueError("Bake result has missing, duplicate or unexpected objects")
    for output, index in zip(outputs, expected):
        if (type(output["index"]) is not int or type(output.get("vertices")) is not int
                or output["vertices"] != vertex_counts[index]):
            raise ValueError("Bake result has an invalid object/vertex count")
        encoding = output.get("encoding", "VERTEX")
        if encoding not in ("VERTEX", "AFFINE") or (encoding == "AFFINE" and (
                request["schema_version"] < 8 or request["objects"][index]["material"]["role"] != "RIGID")):
            raise ValueError("Unsupported cache encoding for this object/schema")
        stored = output.get("stored_frames", frames)
        if type(stored) is not int or stored not in (1, frames):
            raise ValueError("Invalid stored cache frame count")
        if stored != frames and (request["schema_version"] < 6 or fixed_flags is None
                                 or not fixed_flags[index]):
            raise ValueError("Constant cache requires a proven fully fixed input")
        digest = output.get("sha256")
        if result.get("cache_integrity") == 1 and (
                not isinstance(digest, str) or len(digest) != 64
                or any(c not in "0123456789abcdef" for c in digest)):
            raise ValueError("Bake result is missing a valid cache checksum")
    return frames


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_mesh(vertices, triangles, role, name, allow_components=False):
    """Validate input before any native call; preserve the original vertex indexing."""
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int32)
    prefix = f"{name}: "
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3:
        raise ValueError(prefix + "expected at least three vertices")
    if triangles.ndim != 2 or triangles.shape[1] != 3 or not len(triangles):
        raise ValueError(prefix + "expected triangulated faces")
    if not np.isfinite(vertices).all():
        raise ValueError(prefix + "non-finite vertex coordinates")
    if triangles.min() < 0 or triangles.max() >= len(vertices):
        raise ValueError(prefix + "triangle index out of range")
    if len(np.unique(triangles)) != len(vertices):
        raise ValueError(prefix + "loose vertices/edges are unsupported; remove them first")
    canonical = np.sort(triangles, axis=1)
    if len(np.unique(canonical, axis=0)) != len(triangles):
        raise ValueError(prefix + "duplicate triangles")
    p = vertices[triangles]
    cross = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    extent = float(np.max(np.ptp(vertices, axis=0)))
    if extent <= 0 or np.any(np.linalg.norm(cross, axis=1) <= extent**2 * 1e-12):
        raise ValueError(prefix + "degenerate triangles; clean the mesh first")
    edges = {}
    adjacency = [[] for _ in triangles]
    for face, tri in enumerate(triangles):
        for a, b in zip(tri, np.roll(tri, -1)):
            edge = tuple(sorted((int(a), int(b))))
            edges.setdefault(edge, []).append((face, int(a) < int(b)))
    for incident in edges.values():
        if len(incident) > 2:
            raise ValueError(prefix + "non-manifold edge")
        if len(incident) == 2:
            (a, direction_a), (b, direction_b) = incident
            if direction_a == direction_b:
                raise ValueError(prefix + "inconsistent face winding; recalculate normals")
            adjacency[a].append(b)
            adjacency[b].append(a)
        elif role == "RIGID":
            raise ValueError(prefix + "ABD rigid bodies require a closed surface")
    if role == "RIGID":
        seen, pending = set(), [0]
        while pending:
            face = pending.pop()
            if face not in seen:
                seen.add(face)
                pending.extend(adjacency[face])
        if len(seen) != len(triangles) and not allow_components:
            raise ValueError(prefix + "use one connected closed surface per ABD body")
        # Center before integrating to avoid cancellation for translated meshes.
        centered = vertices - vertices.mean(axis=0)
        p = centered[triangles]
        volume = float(np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum() / 6)
        if abs(volume) <= extent**3 * 1e-12:
            raise ValueError(prefix + "ABD surface encloses no usable volume")
        if volume < 0:
            triangles = triangles[:, [0, 2, 1]].copy()
    return np.ascontiguousarray(vertices), np.ascontiguousarray(triangles)


def tetrahedral_surface(tetrahedra):
    """Return consistently oriented boundary faces and their owning tetrahedra."""
    records = {}
    face_slots = ((1, 2, 3), (0, 3, 2), (0, 1, 3), (0, 2, 1))
    for owner, tetrahedron in enumerate(np.asarray(tetrahedra)):
        for slots in face_slots:
            face = tuple(int(tetrahedron[i]) for i in slots)
            key = tuple(sorted(face))
            record = records.get(key)
            if record is None:
                records[key] = [face, owner, 1]
            else:
                record[2] += 1
                if record[2] > 2:
                    raise ValueError("non-manifold tetrahedral face shared by more than two cells")
                start = record[0].index(face[0])
                if record[0][(start + 1) % 3] == face[1]:
                    raise ValueError("tetrahedra overlap or have inconsistent orientation across a shared face")
    boundary = [(record[0], record[1]) for record in records.values() if record[2] == 1]
    if not boundary:
        raise ValueError("tetrahedral mesh has no boundary surface")
    return (np.ascontiguousarray([item[0] for item in boundary], dtype=np.int32),
            np.ascontiguousarray([item[1] for item in boundary], dtype=np.int32))


def validate_tetmesh(vertices, tetrahedra, name):
    """Validate/orient a volume mesh and return its exact boundary surface."""
    vertices = np.asarray(vertices, dtype=np.float64)
    tetrahedra = np.asarray(tetrahedra, dtype=np.int32)
    prefix = f"{name}: "
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4:
        raise ValueError(prefix + "expected at least four tetrahedral vertices")
    if tetrahedra.ndim != 2 or tetrahedra.shape[1] != 4 or not len(tetrahedra):
        raise ValueError(prefix + "expected tetrahedra with four vertex indices")
    if not np.isfinite(vertices).all():
        raise ValueError(prefix + "non-finite tetrahedral vertex coordinates")
    if tetrahedra.min() < 0 or tetrahedra.max() >= len(vertices):
        raise ValueError(prefix + "tetrahedron index out of range")
    canonical = np.sort(tetrahedra, axis=1)
    if np.any(np.diff(canonical, axis=1) == 0):
        raise ValueError(prefix + "tetrahedron contains a repeated vertex")
    if len(np.unique(canonical, axis=0)) != len(tetrahedra):
        raise ValueError(prefix + "duplicate tetrahedra")
    if len(np.unique(tetrahedra)) != len(vertices):
        raise ValueError(prefix + "tetrahedral mesh contains unused vertices")
    points = vertices[tetrahedra]
    matrices = np.stack((points[:, 1] - points[:, 0],
                         points[:, 2] - points[:, 0],
                         points[:, 3] - points[:, 0]), axis=2)
    determinants = np.linalg.det(matrices)
    diagonal = float(np.linalg.norm(np.ptp(vertices, axis=0)))
    tolerance = max(diagonal**3 * 1e-12, np.finfo(np.float64).tiny)
    if diagonal <= 0 or np.any(np.abs(determinants) <= tolerance):
        bad = int(np.argmin(np.abs(determinants)))
        raise ValueError(prefix + f"degenerate tetrahedron {bad}; improve the volume mesh")
    tetrahedra = tetrahedra.copy()
    negative = determinants < 0
    if np.any(negative):
        old_two = tetrahedra[negative, 2].copy()
        tetrahedra[negative, 2] = tetrahedra[negative, 3]
        tetrahedra[negative, 3] = old_two
    surface, owners = tetrahedral_surface(tetrahedra)
    return (np.ascontiguousarray(vertices), np.ascontiguousarray(tetrahedra),
            surface, owners)


class MDDWriter:
    """Stream one frame at a time; never allocate frames x vertices in memory."""

    def __init__(self, path, frame_count, vertex_count, fps):
        self.path = Path(path)
        self.temporary = self.path.with_suffix(".mdd.partial")
        self.frame_count, self.vertex_count = frame_count, vertex_count
        self.written = 0
        self.digest = hashlib.sha256()
        self.file = self.temporary.open("xb")
        self._write(struct.pack(">ii", frame_count, vertex_count))
        self._write((np.arange(frame_count, dtype=np.float64) / fps).astype(">f4").tobytes())

    def _write(self, data):
        self.file.write(data)
        self.digest.update(data)

    def append(self, positions):
        positions = np.asarray(positions)
        if positions.shape != (self.vertex_count, 3) or not np.isfinite(positions).all():
            raise ValueError("Invalid or non-finite simulation output")
        with np.errstate(over="raise", invalid="raise"):
            data = positions.astype(">f4")
        self._write(data.tobytes())
        self.written += 1

    def close(self, commit=False):
        if not self.file.closed:
            self.file.close()
        if commit:
            if self.written != self.frame_count:
                raise ValueError("Incomplete MDD cache")
            os.replace(self.temporary, self.path)


def inspect_mdd(path, frames, vertices):
    path = Path(path)
    with path.open("rb") as stream:
        header = stream.read(8)
    if len(header) != 8 or struct.unpack(">ii", header) != (frames, vertices):
        raise ValueError(f"Invalid cache header: {path.name}")
    if path.stat().st_size != 8 + 4 * frames + 12 * frames * vertices:
        raise ValueError(f"Truncated cache: {path.name}")
