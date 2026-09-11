# SPDX-License-Identifier: Apache-2.0
"""Portable input/cache regressions, independent of Blender and CUDA."""

import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "libuipc_blender" / "protocol.py"
SPEC = importlib.util.spec_from_file_location("uipc_blender_protocol", SOURCE)
protocol = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(protocol)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.vertices = np.array([[0., 0, 0], [1., 0, 0], [0., 1, 0], [0., 0, 1]])
        self.faces = np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]], dtype=np.int32)

    def test_mirrored_rigid_mesh_preserves_vertex_order_and_positive_volume(self):
        mirrored = self.vertices * [-2, 0.5, 1.5] + [1e3, -1e3, 2e3]
        vertices, faces = protocol.validate_mesh(mirrored, self.faces, "RIGID", "body")
        np.testing.assert_array_equal(vertices, mirrored)
        p = (vertices - vertices.mean(axis=0))[faces]
        self.assertGreater(np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum(), 0)

    def test_open_rigid_rejected_but_open_cloth_supported(self):
        with self.assertRaisesRegex(ValueError, "closed"):
            protocol.validate_mesh(self.vertices, self.faces[:-1], "RIGID", "body")
        protocol.validate_mesh(self.vertices, self.faces[:-1], "CLOTH", "cloth")

    def test_invalid_index_nonfinite_degenerate_duplicate_and_winding(self):
        cases = []
        invalid = self.faces.copy()
        invalid[0, 0] = 99
        cases.append((self.vertices, invalid, "index out of range"))
        invalid_vertices = self.vertices.copy()
        invalid_vertices[0, 0] = np.nan
        cases.append((invalid_vertices, self.faces, "non-finite"))
        invalid_vertices = self.vertices.copy()
        invalid_vertices[3] = invalid_vertices[0]
        cases.append((invalid_vertices, self.faces, "degenerate"))
        cases.append((self.vertices, np.concatenate([self.faces, self.faces[:1]]), "duplicate"))
        invalid = self.faces.copy()
        invalid[0] = invalid[0, ::-1]
        cases.append((self.vertices, invalid, "winding"))
        for vertices, faces, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                protocol.validate_mesh(vertices, faces, "CLOTH", "invalid")

    def test_mdd_stream_format_and_truncation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.mdd"
            writer = protocol.MDDWriter(path, 2, 4, 50)
            writer.append(self.vertices)
            self.assertFalse(path.exists())
            writer.append(self.vertices + [0, 0, 0.5])
            writer.close(commit=True)
            protocol.inspect_mdd(path, 2, 4)
            with path.open("rb") as file:
                self.assertEqual(struct.unpack(">ii", file.read(8)), (2, 4))
                np.testing.assert_allclose(np.frombuffer(file.read(8), dtype=">f4"), [0, 0.02])
                frames = np.frombuffer(file.read(), dtype=">f4").reshape(2, 4, 3)
                np.testing.assert_allclose(frames[1], self.vertices + [0, 0, 0.5])
            with path.open("r+b") as file:
                file.truncate(path.stat().st_size - 1)
            with self.assertRaisesRegex(ValueError, "Truncated"):
                protocol.inspect_mdd(path, 2, 4)

    def test_incomplete_and_nonfinite_cache_never_committed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.mdd"
            writer = protocol.MDDWriter(path, 2, 4, 50)
            with self.assertRaises(ValueError):
                writer.append(self.vertices * float("nan"))
            writer.append(self.vertices)
            with self.assertRaisesRegex(ValueError, "Incomplete"):
                writer.close(commit=True)
            self.assertFalse(path.exists())

    def test_fingerprint_detects_values_topology_pins_and_units(self):
        body = {"name": "mesh", "material": {"role": "CLOTH"},
                "vertices": self.vertices, "triangles": self.faces, "matrix": np.eye(4),
                "pins": np.array([0], dtype=np.int32)}
        baseline = protocol.fingerprint({"unit_scale": 1}, [body])
        self.assertNotEqual(baseline, protocol.fingerprint({"unit_scale": 0.01}, [body]))
        for field in ("vertices", "triangles", "matrix", "pins"):
            modified = dict(body)
            modified[field] = body[field].copy()
            modified[field].flat[0] += 1
            self.assertNotEqual(baseline, protocol.fingerprint({"unit_scale": 1}, [modified]))

    def test_atomic_status_retries_windows_reader_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            actual_replace = protocol.os.replace
            calls = 0

            def replace(source, destination):
                nonlocal calls
                calls += 1
                if calls < 3:
                    raise PermissionError("sharing violation")
                actual_replace(source, destination)

            with mock.patch.object(protocol.os, "replace", side_effect=replace), mock.patch.object(protocol.time, "sleep"):
                protocol.atomic_json(path, {"state": "complete"})
            self.assertEqual(calls, 3)
            self.assertEqual(protocol.read_json(path), {"state": "complete"})

    def test_volume_boundary_and_orientation(self):
        vertices, cells, boundary, owners = protocol.validate_tetmesh(
            self.vertices, np.array([[0, 2, 1, 3]]), "tetrahedron")
        self.assertEqual({tuple(sorted(f)) for f in boundary}, {tuple(sorted(f)) for f in self.faces})
        p = vertices[cells[0]]
        self.assertGreater(np.linalg.det(np.stack([p[1]-p[0], p[2]-p[0], p[3]-p[0]], axis=1)), 0)
        np.testing.assert_array_equal(owners, np.zeros(4, dtype=np.int32))
        with self.assertRaisesRegex(ValueError, "repeated"):
            protocol.validate_tetmesh(self.vertices, np.array([[0, 1, 1, 3]]), "invalid")
        with self.assertRaisesRegex(ValueError, "duplicate"):
            protocol.validate_tetmesh(self.vertices, np.array([[0, 1, 2, 3], [0, 1, 2, 3]]), "invalid")

    def test_new_fixed_flag_invalidates_legacy_cache(self):
        body = {"name": "mesh", "material": {"role": "CLOTH"},
                "vertices": self.vertices, "triangles": self.faces, "matrix": np.eye(4),
                "pins": np.array([0], dtype=np.int32)}
        request = {"schema_version": 1, "objects": [{"material": {"role": "CLOTH"}}]}
        old = protocol.fingerprint({}, [body], schema_version=1)
        body["material"].update(fixed=False, young_modulus=100000)
        self.assertEqual(protocol.cache_fingerprint(request, {}, [body]), old)
        body["material"]["fixed"] = True
        self.assertIsNone(protocol.cache_fingerprint(request, {}, [body]))

    def test_robot_drive_invalidates_legacy_cache_and_motion_hash(self):
        body = {"name": "mesh", "material": {"role": "RIGID"},
                "vertices": self.vertices, "triangles": self.faces, "matrix": np.eye(4),
                "pins": np.array([], dtype=np.int32)}
        old = protocol.fingerprint({}, [body], schema_version=2)
        request = {"schema_version": 2, "objects": [{"material": {"role": "RIGID"}}]}
        self.assertEqual(protocol.cache_fingerprint(request, {}, [body]), old)
        body["material"]["drive"] = {"signature": "authored-control-curves"}
        self.assertNotEqual(protocol.cache_fingerprint(request, {}, [body]), old)
        request["schema_version"] = 1
        self.assertIsNone(protocol.cache_fingerprint(request, {}, [body]))
        motion = np.repeat(np.eye(4)[None], 3, axis=0)
        original = protocol.motion_hash(motion)
        motion[1, 2, 3] = 0.01
        self.assertNotEqual(original, protocol.motion_hash(motion))

    def test_driven_compound_requires_closed_components(self):
        vertices = np.concatenate((self.vertices, self.vertices + [2, 0, 0]))
        faces = np.concatenate((self.faces, self.faces + 4))
        with self.assertRaisesRegex(ValueError, "connected"):
            protocol.validate_mesh(vertices, faces, "RIGID", "assembly")
        protocol.validate_mesh(vertices, faces, "RIGID", "assembly", allow_components=True)
        with self.assertRaisesRegex(ValueError, "closed"):
            protocol.validate_mesh(vertices, faces[:-1], "RIGID", "open assembly", allow_components=True)


if __name__ == "__main__":
    unittest.main()
