# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Host wall-time attribution; deliberately adds no CUDA synchronization."""

from contextlib import contextmanager
from functools import wraps
import time


class Timings:
    def __init__(self):
        self.started = time.perf_counter()
        self.phases = {}

    def add(self, name, seconds):
        entry = self.phases.setdefault(name, {"seconds": 0.0, "calls": 0})
        entry["seconds"] += seconds
        entry["calls"] += 1

    @contextmanager
    def measure(self, name):
        start = time.perf_counter()
        try:
            yield
        finally:
            self.add(name, time.perf_counter() - start)

    def report(self):
        return {"schema_version": 1, "total_seconds": time.perf_counter() - self.started,
                "clock": "host_perf_counter_no_added_gpu_sync",
                "phases": {name: dict(entry) for name, entry in self.phases.items()}}


# Diagnostics must not mutate Scene RNA, invalidate a bake, or grow with calls.
_frontend = {}


def clear():
    _frontend.clear()


def frontend_report(scene):
    return {name: dict(entry) for name, entry in _frontend.get(scene.as_pointer(), {}).items()}


def frontend_phase(name):
    def decorate(function):
        @wraps(function)
        def measured(scene, *args, **kwargs):
            start, succeeded = time.perf_counter(), False
            try:
                result = function(scene, *args, **kwargs)
                succeeded = True
                return result
            finally:
                if len(_frontend) >= 32 and scene.as_pointer() not in _frontend:
                    _frontend.clear()
                phases = _frontend.setdefault(scene.as_pointer(), {})
                entry = phases.setdefault(name, {"calls": 0, "seconds": 0.0})
                entry.update(last_seconds=time.perf_counter() - start, succeeded=succeeded)
                entry["seconds"] += entry["last_seconds"]
                entry["calls"] += 1
        return measured
    return decorate
