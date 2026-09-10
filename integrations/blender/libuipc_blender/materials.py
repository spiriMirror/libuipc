# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Portable physical material rules, independent of Blender and native uipc."""

import math


CLOTH_POISSON_FIELDS = ("stretch_poisson", "shear_poisson", "bending_poisson")


def cloth_moduli(material):
    """Old requests use their shared Poisson ratio until independently edited."""
    result = {}
    for channel in ("stretch", "shear", "bending"):
        young = float(material[channel])
        poisson = float(material.get(channel + "_poisson", material["poisson"]))
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
