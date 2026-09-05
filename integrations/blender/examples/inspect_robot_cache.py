# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Read complete frames from a running robot bake without modifying its cache."""

import argparse
import json
from pathlib import Path
import struct

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--object", default="06 Apple 5")
    args = parser.parse_args()
    request = json.loads((args.directory / "request.json").read_text())
    index = next(
        i for i, obj in enumerate(request["objects"]) if obj["name"] == args.object
    )
    path = args.directory / f"object_{index:04d}.mdd"
    if not path.exists():
        path = path.with_suffix(".mdd.partial")
    with path.open("rb") as stream:
        expected, vertices = struct.unpack(">ii", stream.read(8))
        offset = 8 + 4 * expected
        stream.seek(0, 2)
        available = max(0, (stream.tell() - offset) // (vertices * 12))
        stream.seek(offset)
        points = np.frombuffer(
            stream.read(available * vertices * 12), dtype=">f4"
        ).reshape(available, vertices, 3)
    with np.load(args.directory / f"input_{index:04d}.npz", allow_pickle=False) as data:
        matrix = data["matrix"]
    centers = points.mean(axis=1) @ matrix[:3, :3].T + matrix[:3, 3]
    selected = sorted(
        {
            min(n, available)
            for n in (1, 50, 150, 195, 250, 325, 380, 420, 500, available)
            if n > 0
        }
    )
    print(
        json.dumps(
            {
                "object": args.object,
                "frames": available,
                "centers": {str(n): centers[n - 1].tolist() for n in selected},
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
