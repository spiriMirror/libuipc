# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Portable physical material rules, independent of Blender and native uipc."""

import math


CLOTH_POISSON_FIELDS = ("stretch_poisson", "shear_poisson", "bending_poisson")
PRESET_FIELDS = {
    "CLOTH": ("density", "thickness", "stretch", "shear", "bending", *CLOTH_POISSON_FIELDS,
              "strain_rate", "self_collision"),
    "RIGID": ("density", "rigidity"),
    "FEM": ("density", "thickness", "young_modulus", "poisson", "self_collision"),
    "ROD": ("density", "thickness", "rod_stretch", "rod_bending", "self_collision"),
}


def cloth_moduli(material):
    """Old requests use their shared Poisson ratio until independently edited."""
    result = {}
    for channel in ("stretch", "shear", "bending"):
        young = float(material[channel])
        value = material.get(channel + "_poisson", material.get("poisson"))
        if value is None:
            raise ValueError(f"{channel}: a Poisson ratio is required")
        poisson = float(value)
        if not math.isfinite(young) or young < 0 or (channel != "bending" and young == 0):
            raise ValueError(f"{channel}: Young's modulus must be finite and positive (bending may be zero)")
        if not math.isfinite(poisson) or not 0 <= poisson < 0.5:
            raise ValueError(f"{channel}: Poisson ratio must be in [0, 0.5)")
        result[channel] = (young, poisson)
    return result


def cloth_stiffness(material):
    """Match the native membrane/bending conversions, before geometric weights."""
    values = cloth_moduli(material)
    radius = float(material["thickness"])
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("Thickness radius must be finite and positive")
    stretch, ns = values["stretch"]
    shear, ng = values["shear"]
    bending, nb = values["bending"]
    return {"stretch": stretch * (2 * radius) / (1 - ns * ns),
            "shear": shear / (2 * (1 + ng)),
            "bending": bending * (2 * radius)**3 / (12 * (1 - nb * nb))}


def contact_label(value):
    if not isinstance(value, str) or "\0" in value or len(value) > 120:
        raise ValueError("Contact material must be a name of at most 120 characters")
    return value.strip() or "Default"


def normalize_contact_pairs(pairs, labels=None):
    if not isinstance(pairs, list):
        raise ValueError("Contact pairs must be a list")
    result, seen = [], set()
    for pair in pairs:
        if not isinstance(pair, dict) or set(pair) != {"material_a", "material_b", "friction", "resistance", "enabled"}:
            raise ValueError("Contact pair fields are incomplete or unknown")
        a, b = sorted((contact_label(pair["material_a"]), contact_label(pair["material_b"])))
        if (a, b) in seen:
            raise ValueError(f"Duplicate contact pair: {a} / {b}")
        seen.add((a, b))
        if labels is not None and ({a, b} - set(labels) - {"Default"}):
            raise ValueError(f"Contact pair {a} / {b} refers to an unassigned material")
        for key, minimum in (("friction", 0), ("resistance", 1)):
            value = pair[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
                raise ValueError(f"Contact {key} must be finite and >= {minimum}")
        if type(pair["enabled"]) is not bool:
            raise ValueError("Contact enabled must be boolean")
        result.append({**pair, "material_a": a, "material_b": b})
    return sorted(result, key=lambda p: (p["material_a"], p["material_b"]))


def build_contact_plan(settings, bodies):
    """Explicit material pairs override defaults, never an assembly exclusion."""
    labels = {contact_label(b["material"].get("contact_material", "")) for b in bodies}
    pairs = normalize_contact_pairs(settings.get("contact_pairs", []), labels)
    overrides = {(p["material_a"], p["material_b"]): p for p in pairs}
    groups = [{"material": "Default", "assembly": "", "driven": False, "friction": None}]
    ids, assembly_friction, assignments = {("material", "Default"): 0}, {}, []
    for body in bodies:
        material = body["material"]
        label = contact_label(material.get("contact_material", ""))
        drive = material.get("drive")
        if drive:
            assembly = drive["group"]
            friction = drive["friction"]
            if not math.isfinite(friction) or friction < 0:
                raise ValueError("Robot friction must be finite and non-negative")
            if assembly and assembly_friction.setdefault(assembly, friction) != friction:
                raise ValueError("Bodies in a robot collision group must share its friction")
            key = ("drive", assembly or "body:" + body["name"], label)
        else:
            assembly, friction = "", None
            key = ("material", label)
        if key not in ids:
            ids[key] = len(groups)
            groups.append({"material": label, "assembly": assembly,
                           "driven": bool(drive), "friction": friction})
        assignments.append(ids[key])
    models = []
    for i, a in enumerate(groups):
        for j in range(i, len(groups)):
            b = groups[j]
            override = overrides.get(tuple(sorted((a["material"], b["material"]))))
            friction, resistance, enabled = settings["friction"], settings["resistance"], True
            if a["assembly"] and a["assembly"] == b["assembly"]:
                friction, resistance, enabled = 0.0, 0.0, False
            elif override is not None:
                friction, resistance, enabled = override["friction"], override["resistance"], override["enabled"]
            elif a["driven"] != b["driven"]:
                friction = a["friction"] if a["driven"] else b["friction"]
            models.append({"a": i, "b": j, "friction": friction, "resistance": resistance, "enabled": enabled})
    return {"groups": groups, "assignments": assignments, "models": models}


def validate_preset(payload):
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError("Unsupported physical material preset")
    role, values = payload.get("role"), payload.get("values")
    if role not in PRESET_FIELDS or not isinstance(values, dict) or set(values) != set(PRESET_FIELDS[role]):
        raise ValueError("Physical material preset has invalid fields or role")
    for key, value in values.items():
        if key == "self_collision":
            if type(value) is not bool:
                raise ValueError("Preset self_collision must be boolean")
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"Preset {key} must be a finite number")
        if key.endswith("poisson"):
            if not 0 <= value <= 0.499 + 1e-7:
                raise ValueError(f"Preset {key} must be in [0, 0.499]")
        else:
            minimum = 0 if key in ("bending", "rod_bending") else 1 if key == "rigidity" else 1e-7 if key == "thickness" else 1e-6
            if value < minimum * (1 - 1e-6):
                raise ValueError(f"Preset {key} must be >= {minimum}")
    return {"schema_version": 1, "role": role, "values": dict(values)}
