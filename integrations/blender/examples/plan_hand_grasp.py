# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Offline joint-space grasp planning from the sample 87 native URDF export."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares


class Hand:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.model = json.loads((self.directory / "robot.json").read_text())
        self.links = {
            link["name"]: dict(
                np.load(self.directory / f"link_{link['index']:03d}.npz")
            )
            for link in self.model["links"]
        }
        self.joints = {j["name"]: j for j in self.model["joints"]}
        self.origins = {}
        for name, j in self.joints.items():
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_euler("xyz", j["rpy"]).as_matrix()
            matrix[:3, 3] = j["xyz"]
            self.origins[name] = matrix
        expected = self.model["fk_test_poses"]
        actual = self.fk(self.model["fk_test_angles"])
        assert max(np.abs(actual[n] - expected[n]).max() for n in expected) < 1e-12

    def fk(self, angles):
        poses = {self.model["root"]: np.eye(4)}
        pending = list(self.joints)
        while pending:
            previous = len(pending)
            for name in pending[:]:
                j = self.joints[name]
                if j["parent"] not in poses:
                    continue
                rotation = np.eye(4)
                axis = np.array(j["axis"])
                rotation[:3, :3] = Rotation.from_rotvec(
                    axis / np.linalg.norm(axis) * angles.get(name, 0)
                ).as_matrix()
                poses[j["child"]] = poses[j["parent"]] @ self.origins[name] @ rotation
                pending.remove(name)
            if len(pending) == previous:
                raise ValueError("Disconnected URDF graph")
        return poses

    def points(self, angles):
        poses = self.fk(angles)
        return {
            name: body["vertices"] @ poses[name][:3, :3].T + poses[name][:3, 3]
            for name, body in self.links.items()
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose", type=Path)
    parser.add_argument("--solve", type=Path)
    args = parser.parse_args()
    hand = Hand(args.model)
    if args.solve:
        result = solve_grasp(hand)
        args.solve.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
        return
    angles = json.loads(args.pose.read_text())["angle_controller"] if args.pose else {}
    points = hand.points(angles)
    for name, v in points.items():
        tris = hand.links[name]["triangles"]
        vertices = hand.links[name]["vertices"]
        p = vertices[tris]
        volume = float(
            np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum() / 6
        )
        print(
            name,
            "bounds",
            v.min(axis=0),
            v.max(axis=0),
            "center",
            v.mean(axis=0),
            "volume",
            volume,
        )


def solve_grasp(hand):
    active = ["0", "1", "2", "3", "4", "5", "6", "7", "12", "13", "14", "15"]
    tips = ["fingertip", "fingertip_2", "thumb_fingertip"]
    zeros = hand.fk({})
    world0 = hand.points({})
    pads = {}
    normals = {}
    for name in tips:
        if name.startswith("thumb"):
            wanted = np.array([-0.073, 0.117, -0.022])
            normal = np.array([1, 0, 0])
        else:
            wanted = np.array([0.119, world0[name][:, 1].mean(), -0.0295])
            normal = np.array([0, 0, -1])
        vertex = np.argmin(np.linalg.norm(world0[name] - wanted, axis=1))
        pads[name] = hand.links[name]["vertices"][vertex]
        normals[name] = zeros[name][:3, :3].T @ normal
    directions = np.array([[0.85, 0.52, -0.05], [0.85, -0.52, -0.05], [-1, 0, -0.08]])
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    low = np.array([hand.joints[name]["limits"][0] for name in active])
    high = np.array([hand.joints[name]["limits"][1] for name in active])
    initial = np.array([0.0, 0.9, 0.7, 0.3, 0.0, 0.9, 0.7, 0.3, 1.25, 0.9, 0.3, 0.4])
    sampled = {
        name: link["vertices"][:: max(1, len(link["vertices"]) // 40)]
        for name, link in hand.links.items()
    }

    def fit(seed, radius, center=None):
        free = center is None

        def residual(values):
            angles = dict(zip(active, values[: len(active)]))
            point = values[-3:] if free else center
            poses = hand.fk(angles)
            errors = []
            for i, name in enumerate(tips):
                pose = poses[name]
                pad = pose[:3, :3] @ pads[name] + pose[:3, 3]
                normal = pose[:3, :3] @ normals[name]
                errors.extend((pad - point - directions[i] * radius) * 200)
                errors.extend((normal + directions[i]) * 0.35)
            for name, vertices in sampled.items():
                pose = poses[name]
                positions = vertices @ pose[:3, :3].T + pose[:3, 3]
                # Avoid the apple interior except a small commanded preload at
                # the pad. Real IPC contact resolves the commanded overlap.
                errors.extend(
                    np.minimum(np.linalg.norm(positions - point, axis=1) - 0.037, 0)
                    * 120
                )
                if name in tips:
                    errors.extend(
                        np.minimum(positions[:, 2] - (point[2] - 0.018), 0) * 30
                    )
            if free:
                errors.extend((point - np.array([-0.025, -0.021, -0.115])) * 3)
            errors.extend((values[: len(active)] - initial) * 0.004)
            return np.array(errors)

        lower = np.r_[low, [-0.06, -0.05, -0.16]] if free else low
        upper = np.r_[high, [0.015, 0.025, -0.075]] if free else high
        answer = least_squares(
            residual,
            seed,
            bounds=(lower, upper),
            max_nfev=450,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        return answer

    closed = fit(np.r_[initial, [-0.025, -0.021, -0.115]], 0.038)
    center = closed.x[-3:]
    opened = fit(closed.x[: len(active)], 0.064, center)
    closed_angles = {name: 0.0 for name in hand.joints}
    open_angles = closed_angles.copy()
    closed_angles.update(zip(active, closed.x[: len(active)].tolist()))
    open_angles.update(zip(active, opened.x.tolist()))
    pad_world = {}
    for label, angles in (("open", open_angles), ("closed", closed_angles)):
        poses = hand.fk(angles)
        pad_world[label] = {
            name: (poses[name][:3, :3] @ pads[name] + poses[name][:3, 3]).tolist()
            for name in tips
        }
    return {
        "grasp_center_hand": center.tolist(),
        "open": open_angles,
        "closed": closed_angles,
        "pad_local": {n: p.tolist() for n, p in pads.items()},
        "pad_world": pad_world,
        "closed_cost": float(closed.cost),
        "open_cost": float(opened.cost),
        "model": "sample 87 URDF collision geometry; three-pad opposition, free middle/third unused joints remain zero",
    }


if __name__ == "__main__":
    main()
