# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Persistent object identities; validation never repairs ambiguous copied IDs."""

import uuid

KEY = "uipc_object_id"


def object_id(obj):
    value = obj.get(KEY)
    return value if isinstance(value, str) and len(value) == 32 and all(c in "0123456789abcdef" for c in value) else None


def related_objects(scene):
    physical = [o for o in scene.objects if o.uipc_body.role != "NONE"]
    related = set(physical)
    for obj in physical:
        current = obj.parent
        while current:
            related.add(current)
            current = current.parent
        if obj.uipc_body.driven:
            current = obj.uipc_body.drive_target
            while current:
                related.add(current)
                current = current.parent
    return physical, related


def ensure_scene_ids(scene):
    _, objects = related_objects(scene)
    seen = {}
    for obj in objects:
        value = object_id(obj)
        if value and value in seen:
            raise ValueError(f"Copied simulation ID on '{seen[value].name}' and '{obj.name}'; select the duplicate and Assign New Simulation ID")
        if value:
            seen[value] = obj
        if obj.library:
            raise ValueError(f"{obj.name}: make simulation objects/controllers local first")
    for obj in objects:
        if not object_id(obj):
            obj[KEY] = uuid.uuid4().hex


def resolve_object(scene, entry, schema):
    if schema < 5:
        obj = scene.objects.get(entry["name"])
        if obj is None:
            raise ValueError(f"Legacy cached object was removed or renamed: {entry['name']}")
        return obj
    matches = [o for o in scene.objects if o.uipc_body.role != "NONE" and object_id(o) == entry.get("id")]
    if len(matches) != 1:
        raise ValueError(f"Cached object ID is missing or duplicated: {entry['name']}")
    return matches[0]
