# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Immutable render jobs and checked per-frame receipts, without Blender imports."""

import glob
import os
from pathlib import Path
import struct
import time
import zlib

if __package__:
    from .protocol import file_sha256, read_json
else:
    from protocol import file_sha256, read_json


def png_info(path):
    """Check dimensions, every chunk CRC and a complete ending in bounded memory."""
    path = Path(path)
    with path.open("rb") as stream:
        if stream.read(8) != b"\x89PNG\r\n\x1a\n":
            raise ValueError(f"Invalid PNG signature: {path}")
        dimensions, data_seen = None, False
        while True:
            header = stream.read(8)
            if len(header) != 8:
                raise ValueError(f"Incomplete PNG: {path}")
            length, kind = struct.unpack(">I4s", header)
            if kind == b"IHDR" and (length != 13 or dimensions is not None):
                raise ValueError("Invalid PNG image header")
            checksum, remaining = zlib.crc32(kind), length
            while remaining:
                block = stream.read(min(remaining, 1024 * 1024))
                if not block:
                    raise ValueError(f"Truncated PNG chunk: {path}")
                if kind == b"IHDR":
                    dimensions = struct.unpack(">II", block[:8])
                checksum = zlib.crc32(block, checksum)
                remaining -= len(block)
            crc = stream.read(4)
            if len(crc) != 4 or checksum != struct.unpack(">I", crc)[0]:
                raise ValueError(f"PNG CRC mismatch: {path}")
            data_seen |= kind == b"IDAT"
            if kind == b"IEND":
                if length or not data_seen or dimensions is None or stream.read(1):
                    raise ValueError(f"Invalid PNG ending: {path}")
                return dimensions


def replace_with_retry(source, destination):
    for attempt in range(100):
        try:
            os.replace(source, destination)
            return
        except PermissionError:
            if attempt == 99:
                raise
            time.sleep(.02)


def dependency_paths(path):
    pattern = path.replace("<UDIM>", "[0-9][0-9][0-9][0-9]")
    return sorted(p for p in glob.glob(pattern) if Path(p).is_file()) if "<UDIM>" in path else ([path] if Path(path).is_file() else [])


def dependency_record(path):
    path = str(Path(path).absolute())
    files = dependency_paths(path)
    return {"path": path, "files": [{"path": p, "sha256": file_sha256(p)} for p in files]}


def dependency_stamps(directory, manifest):
    def stamp(path):
        info = Path(path).stat()
        return path, info.st_size, info.st_mtime_ns
    result = [stamp(str(Path(directory) / "scene.blend"))]
    for record in manifest["dependencies"]:
        result.append((record["path"], tuple(stamp(p) for p in dependency_paths(record["path"]))))
    return tuple(result)


def validate_job(directory, verify_files=True):
    directory = Path(directory).resolve()
    manifest = read_json(directory / "render_manifest.json")
    if manifest.get("schema_version") != 1 or Path(manifest["directory"]).resolve() != directory:
        raise ValueError("Unsupported or relocated render job; create a new queue in its destination")
    if file_sha256(directory / "scene.blend") != manifest["snapshot_sha256"]:
        raise ValueError("Render snapshot changed; create a new queue")
    if (not manifest.get("shots") or len(manifest["resolution"]) != 2
            or any(type(v) is not int or v < 1 for v in manifest["resolution"])):
        raise ValueError("Invalid render job settings")
    for shot in manifest["shots"]:
        if (not isinstance(shot.get("camera"), str) or not shot["camera"]
                or type(shot.get("first")) is not int or type(shot.get("last")) is not int
                or shot["last"] < shot["first"]):
            raise ValueError("Invalid render shot")
    if verify_files:
        for record in manifest["dependencies"]:
            if dependency_record(record["path"]) != record:
                raise ValueError(f"Render dependency changed: {record['path']}; create a new queue")
    return manifest, file_sha256(directory / "render_manifest.json")


def frame_paths(directory, camera_index, frame):
    folder = Path(directory) / f"camera_{camera_index:03d}"
    return folder / f"frame_{frame:06d}.png", folder / f"frame_{frame:06d}.json"


def completed_frame(directory, camera_index, frame, signature, resolution):
    image, receipt = frame_paths(directory, camera_index, frame)
    try:
        record = read_json(receipt)
        return (record.get("input_guard") is True and record["job_signature"] == signature and record["camera_index"] == camera_index
                and record["frame"] == frame and record["sha256"] == file_sha256(image)
                and png_info(image) == tuple(resolution))
    except (OSError, ValueError, KeyError):
        return False
