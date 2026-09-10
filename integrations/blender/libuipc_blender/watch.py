# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Relevant dependency updates and a cheap raw-input gate before full validation."""

from pathlib import Path

import bpy

from .identity import related_objects, object_id
from .protocol import fingerprint, MODIFIER_NAME

_states = {}
_members = {}


def clear():
    _states.clear()
    _members.clear()


def relevant_update(scene, updates):
    physical, related = related_objects(scene)
    membership = (tuple(sorted(o.as_pointer() for o in related)), scene.frame_start, scene.frame_end,
                  scene.render.fps, scene.render.fps_base, scene.unit_settings.scale_length)
    key = scene.as_pointer()
    if _members.get(key) != membership:
        _members[key] = membership
        return True
    meshes = {o.data for o in physical if o.type == "MESH"}
    actions = {o.animation_data.action for o in related if o.animation_data and o.animation_data.action}
    for update in updates:
        item = getattr(update.id, "original", update.id)
        if isinstance(item, bpy.types.Mesh) and item in meshes:
            return True
        if isinstance(item, bpy.types.Object) and item in related:
            return True
        if isinstance(item, bpy.types.Action) and item in actions:
            return True
    return False


def check(scene):
    from . import bridge
    settings, bodies = bridge.collect_scene(scene, validate_geometry=False)
    controls, files = [], []
    for obj in sorted((o for o in scene.objects if o.uipc_body.role != "NONE"), key=lambda o: object_id(o) or o.name):
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if modifier:
            controls.append((object_id(obj) or obj.name, list(obj.modifiers).index(modifier),
                             tuple(tuple(getattr(modifier, k)) if k == "flip_axis" else getattr(modifier, k)
                                   for k in bridge.CACHE_PROPERTIES)))
            path = Path(bpy.path.abspath(modifier.filepath))
            stat = path.stat()
            files.append((str(path), stat.st_size, stat.st_mtime_ns))
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    for name in ("request.json", "result.json"):
        stat = (directory / name).stat()
        files.append((str(directory / name), stat.st_size, stat.st_mtime_ns))
    token = (scene.uipc_settings.last_bake, fingerprint(settings, bodies), tuple(controls), tuple(files))
    key, previous = scene.as_pointer(), _states.get(scene.as_pointer())
    if previous == token:
        return False
    _states[key] = token
    bridge.check_cache(scene, verify_data=previous is not None and previous[-1] != token[-1])
    return True
