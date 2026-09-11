# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Streaming observational diagnostics; never changes solver/material state."""

import hashlib
import json
import os
from pathlib import Path

import numpy as np

from protocol import atomic_json


class QualityRecorder:
    def __init__(self, directory, request):
        self.directory = Path(directory)
        self.request = request
        self.fps = request["settings"]["fps"]
        self.start = request["settings"]["frame_start"]
        self.previous = {}
        self.velocities = {}
        self.objects = {}
        self.temporary = self.directory / "quality_frames.jsonl.partial"
        self.file = self.temporary.open("xb")
        self.digest = hashlib.sha256()
        self.solver = {"steps": 0, "max_newton_iterations": 0, "max_linear_solver_iterations": 0,
                       "max_line_search_trials": 0, "nonconverged_steps": 0,
                       "limit_frames": [], "minimum_line_search_alpha": 1.0,
                       "minimum_ccd_toi": 1.0}
        self.limit_frames = set()

    def _record(self, data):
        line = (json.dumps(data, allow_nan=False, separators=(",", ":")) + "\n").encode("utf-8")
        self.file.write(line)
        self.digest.update(line)

    def record_solver(self, frame, substep, stats):
        self.solver["steps"] += 1
        for source in ("newton_iterations", "linear_solver_iterations", "line_search_trials"):
            self.solver["max_" + source] = max(self.solver["max_" + source], stats.get(source, 0))
        if stats.get("converged") is False:
            self.solver["nonconverged_steps"] += 1
        if stats.get("hit_newton_limit") or stats.get("hit_line_search_limit"):
            self.limit_frames.add(frame)
        for source, destination in (("last_line_search_alpha", "minimum_line_search_alpha"),
                                    ("last_ccd_toi", "minimum_ccd_toi")):
            self.solver[destination] = min(self.solver[destination], stats.get(source, 1.0))
        self._record({"kind": "solver", **stats, "native_frame": stats.get("frame"),
                      "frame": frame, "substep": substep})

    def record_object(self, index, frame, points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
            raise ValueError("Quality sampling received invalid world positions")
        entry = self.objects.get(index)
        if entry is None:
            body = self.request["objects"][index]
            entry = {"index": index, "name": body["name"], "role": body["material"]["role"],
                     "max_speed": 0.0, "speed_frame": frame, "speed_vertex": 0,
                     "max_acceleration": 0.0, "acceleration_frame": frame, "acceleration_vertex": 0,
                     "final_rms_speed": 0.0, "samples": 0}
            self.objects[index] = entry
        row = {"kind": "object", "index": index, "frame": frame,
               "max_speed": 0.0, "rms_speed": 0.0, "max_acceleration": None}
        if index in self.previous:
            velocity = (points - self.previous[index]) * self.fps
            speed = np.linalg.norm(velocity, axis=1)
            vertex = int(np.argmax(speed))
            row["max_speed"] = float(speed[vertex])
            row["rms_speed"] = float(np.sqrt(np.mean(speed**2)))
            if row["max_speed"] > entry["max_speed"]:
                entry.update(max_speed=row["max_speed"], speed_frame=frame, speed_vertex=vertex)
            if index in self.velocities:
                acceleration = np.linalg.norm((velocity - self.velocities[index]) * self.fps, axis=1)
                vertex = int(np.argmax(acceleration))
                row["max_acceleration"] = float(acceleration[vertex])
                if row["max_acceleration"] > entry["max_acceleration"]:
                    entry.update(max_acceleration=row["max_acceleration"], acceleration_frame=frame,
                                 acceleration_vertex=vertex)
            self.velocities[index] = velocity
        # Native position views can change during the next advance; own this history.
        self.previous[index] = points.copy()
        entry["final_rms_speed"] = row["rms_speed"]
        entry["samples"] += 1
        self._record(row)

    def finish(self):
        self.close()
        os.replace(self.temporary, self.directory / "quality_frames.jsonl")
        self.solver["limit_frames"] = sorted(self.limit_frames)
        report = {"schema_version": 1, "fingerprint": self.request["fingerprint"],
                  "frame_start": self.start, "frame_end": self.request["settings"]["frame_end"],
                  "fps": self.fps, "sampling": "world-space output-frame finite differences",
                  "units": {"speed": "m/s", "acceleration": "m/s^2"},
                  "note": "Peaks are diagnostic observations, not proof of physical error or convergence.",
                  "series_sha256": self.digest.hexdigest(), "solver": self.solver,
                  "objects": [self.objects[i] for i in sorted(self.objects)]}
        atomic_json(self.directory / "quality_report.json", report)
        return report

    def record_stationary(self, index, frame):
        """After the first sample, proven fixed bodies need no N-vertex history."""
        entry = self.objects[index]
        if entry["max_speed"] or entry["max_acceleration"]:
            raise ValueError("Cannot mark a moving diagnostic history as stationary")
        self.previous.pop(index, None)
        self.velocities.pop(index, None)
        self._record({"kind": "object", "index": index, "frame": frame,
                      "max_speed": 0.0, "rms_speed": 0.0,
                      "max_acceleration": 0.0 if entry["samples"] >= 2 else None})
        entry["samples"] += 1

    def close(self):
        if not self.file.closed:
            self.file.close()
