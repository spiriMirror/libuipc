# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from protocol import fully_fixed, validate_result, fingerprint, cache_fingerprint
from quality import QualityRecorder


class FixedCacheTests(unittest.TestCase):
    def test_only_proven_fixed_geometry_qualifies(self):
        body = {"vertices": np.eye(3), "pins": np.array([0,1,2]), "material": {"role": "CLOTH"}}
        for role in ("CLOTH", "FEM"):
            body["material"]["role"] = role
            self.assertTrue(fully_fixed(body))
        for pins in ([0,1], [0,1,1], [1,2,3], [-1,0,1], []):
            self.assertFalse(fully_fixed({**body, "pins": np.array(pins)}))
        body["material"] = {"role": "RIGID"}
        self.assertFalse(fully_fixed(body))
        body["material"]["fixed"] = True
        self.assertTrue(fully_fixed(body))
        body["material"]["drive"] = {}
        self.assertFalse(fully_fixed(body))

    def test_counts_cannot_forge_fixed_or_legacy_output(self):
        request = {"schema_version": 6, "fingerprint": "test",
                   "settings": {"frame_start": 7, "frame_end": 9},
                   "objects": [{"material": {"role": "CLOTH"}}]}
        result = {"schema_version": 6, "fingerprint": "test", "frames": 3,
                  "objects": [{"index": 0, "vertices": 3, "stored_frames": 1}]}
        self.assertEqual(validate_result(request, result, [3], [True]), 3)
        for flags in ([False], None):
            with self.assertRaises(ValueError):
                validate_result(request, result, [3], flags)
        for count in (0, 2, 4, True, 1.0, "1"):
            invalid = copy.deepcopy(result)
            invalid["objects"][0]["stored_frames"] = count
            with self.assertRaises(ValueError):
                validate_result(request, invalid, [3], [True])
        request["schema_version"] = result["schema_version"] = 5
        with self.assertRaises(ValueError):
            validate_result(request, result, [3], [True])
        del result["objects"][0]["stored_frames"]
        self.assertEqual(validate_result(request, result, [3]), 3)

    def test_stationary_diagnostics_match_dense_byte_for_byte(self):
        request = {"fingerprint": "test", "settings": {"frame_start": 7, "frame_end": 17, "fps": 30},
                   "objects": [{"name": "Fixed", "material": {"role": "CLOTH"}}]}
        with tempfile.TemporaryDirectory() as temporary:
            reports, rows = [], []
            for mode in ("dense", "constant"):
                directory = Path(temporary) / mode
                directory.mkdir()
                quality = QualityRecorder(directory, request)
                for frame in range(7,18):
                    if mode == "constant" and frame > 7:
                        quality.record_stationary(0, frame)
                        self.assertFalse(quality.previous or quality.velocities)
                    else:
                        quality.record_object(0, frame, np.eye(3))
                reports.append(quality.finish())
                rows.append((directory / "quality_frames.jsonl").read_bytes())
            self.assertEqual(reports[0], reports[1])
            self.assertEqual(rows[0], rows[1])

    def test_schema_five_inputs_keep_their_fingerprint(self):
        body = {"id": "stable", "name": "Fixed", "material": {"role": "RIGID", "fixed": True},
                "vertices": np.eye(3), "triangles": [[0,1,2]], "matrix": np.eye(4), "pins": []}
        old = fingerprint({}, [body], 5)
        self.assertEqual(old, fingerprint({}, [body], 6))
        self.assertEqual(old, cache_fingerprint({"schema_version": 5}, {}, [body]))
