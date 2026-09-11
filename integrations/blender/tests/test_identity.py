# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Schema-five stable identities and legacy-name compatibility."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from protocol import fingerprint, cache_fingerprint, match_bodies


class IdentityTests(unittest.TestCase):
    def body(self, identifier, name):
        return {"id": identifier, "name": name, "material": {"role": "RIGID"},
                "vertices": np.eye(3), "triangles": np.array([[0, 1, 2]]),
                "matrix": np.eye(4), "pins": np.array([]), "tetrahedra": np.empty((0, 4))}

    def test_rename_and_name_order_do_not_change_new_fingerprint(self):
        a, b = self.body("a", "A"), self.body("b", "B")
        expected = fingerprint({}, [a, b])
        renamed = [{**b, "name": "First"}, {**a, "name": "Last"}]
        self.assertEqual(fingerprint({}, renamed), expected)
        request = {"schema_version": 5, "objects": [a, b]}
        self.assertEqual([x["id"] for x in match_bodies(request, renamed)], ["a", "b"])

    def test_copied_or_missing_ids_are_rejected(self):
        body = self.body("a", "A")
        request = {"schema_version": 5, "objects": [body]}
        for bodies in ([], [body, {**body, "name": "Copy"}], [{**body, "id": ""}]):
            with self.assertRaises(ValueError):
                match_bodies(request, bodies)

    def test_legacy_names_and_motion_signatures_stay_compatible(self):
        body = self.body("a", "A")
        body["material"]["drive"] = {"target": "Old control", "signature": "name hash"}
        old = fingerprint({}, [body], 4)
        body["material"]["drive"].update(target_id="target", stable_signature="id hash")
        self.assertEqual(cache_fingerprint({"schema_version": 4}, {}, [body]), old)
        new = fingerprint({}, [body])
        body["material"]["drive"].update(target="Renamed", signature="other name hash")
        self.assertEqual(fingerprint({}, [body]), new)
        self.assertNotEqual(fingerprint({}, [body], 4), old)


if __name__ == "__main__":
    unittest.main()
