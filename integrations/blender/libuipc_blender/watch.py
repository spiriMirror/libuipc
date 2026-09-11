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
    from .affine_playback import is_affine, proxy_object
    related = set(related)
    groups = set()
    for obj in physical:
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if is_affine(modifier):
            groups.add(modifier.node_group)
            try:
                related.add(proxy_object(modifier))
            except ValueError:
                return True
    membership = (tuple(sorted(o.as_pointer() for o in related)), scene.frame_start, scene.frame_end,
                  scene.render.fps, scene.render.fps_base, scene.unit_settings.scale_length)
    key = scene.as_pointer()
    if _members.get(key) != membership:
        _members[key] = membership
        return True
    meshes = {o.data for o in related if o.type == "MESH"}
    actions = {o.animation_data.action for o in related if o.animation_data and o.animation_data.action}
    for update in updates:
        item = getattr(update.id, "original", update.id)
        if isinstance(item, bpy.types.Mesh) and item in meshes:
            return True
        if isinstance(item, bpy.types.Object) and item in related:
            return True
        if isinstance(item, bpy.types.Action) and item in actions:
            return True
        if isinstance(item, bpy.types.NodeTree) and item in groups:
            return True
    return False


def check(scene):
    from . import bridge
    settings, bodies = bridge.collect_scene(scene, validate_geometry=False)
    controls, files = [], []
    for obj in sorted((o for o in scene.objects if o.uipc_body.role != "NONE"), key=lambda o: object_id(o) or o.name):
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if modifier:
            from .affine_playback import is_affine, validate_graph, playback_modifier
            if is_affine(modifier):
                proxy = validate_graph(modifier)
                controls.append(("affine", modifier.node_group.as_pointer(), proxy.as_pointer(),
                                 modifier.show_viewport, modifier.show_render,
                                 modifier.node_group.get("uipc_source_id"),
                                 modifier.node_group.get("uipc_cache_fingerprint"),
                                 proxy.get("uipc_source_id"), proxy.get("uipc_cache_fingerprint")))
            cached = playback_modifier(modifier)
            controls.append((object_id(obj) or obj.name, list(obj.modifiers).index(modifier),
                             tuple(tuple(getattr(cached, k)) if k == "flip_axis" else getattr(cached, k)
                                   for k in bridge.CACHE_PROPERTIES)))
            path = Path(bpy.path.abspath(cached.filepath))
            stat = path.stat()
            files.append((str(path), stat.st_size, stat.st_mtime_ns))
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    for name in ("request.json", "result.json"):
        stat = (directory / name).stat()
        files.append((str(directory / name), stat.st_size, stat.st_mtime_ns))
    token = (scene.uipc_settings.last_bake, fingerprint(settings, bodies), tuple(controls), tuple(files))
    key, previous = scene.as_pointer(), _states.get(scene.as_pointer())
    if previous is not None and previous["token"] == token:
        if previous["error"]:
            raise ValueError(previous["error"])
        return False
    try:
        bridge.check_cache(scene, verify_data=previous is None or bool(previous["error"])
                           or previous["token"][-1] != token[-1])
    except (OSError, ValueError) as error:
        _states[key] = {"token": token, "error": str(error)}
        raise
    _states[key] = {"token": token, "error": None}
    return True
