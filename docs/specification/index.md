# Libuipc Specification

This document, referred to as the "Libuipc Specification" defines the behavior of the library.

[TOC]

## Coordinates

The `libuipc` uses 3D coordinates to represent points in space. The coordinates are represented as a tuple of three real numbers `(x, y, z)` with units in meters. The convention used is the right-handed coordinate system.

Although the `libuipc` itself does not depend on the specific meaning of the `XYZ` axes, during development and samples creation, we prefer the following conventions:

- The positive X-axis points rightward
- The positive Y-axis points upward
- The positive Z-axis points backward

<p align="center">
    <img src="../media/coordinates.png" alt="Coordinate System" width="30%" />
</p>

Of course you can take the Z-axis as the Up-axis, only changing the gravity direction in the scene configuration.

## Unit System

The `libuipc` uses the International System of Units (SI) for all measurements. The base units used in the library are as follows:

- Length: meters ($m$)
- Mass: kilograms ($kg$)
- Time: seconds ($s$)

Other units are derived from these base units. For example, velocity is measured in meters per second ($m/s$), and force is measured in Newtons ($N$), where $1N = 1kg \cdot m/s^2$.

## Geometry Orientation

In `libuipc`, the orientation of geometric primitives such as triangles and tetrahedra is crucial for physics quantity calculations, e.g. volume, mass, inertia, etc.

### Triangle Orientation

In `libuipc`:

- Open or closed triangle meshes(trimesh) can describe shell-like structures, such as cloth, paper, etc.
- Closed triangle meshes can describe the surface of affine bodies. `libuipc` uses Green's theorem to compute volumetric information.
- The normal direction of a triangle in a mesh is determined by the triangle's topology index (`topo:<Vector3i>` attribute on triangles).

The right-hand rule (counterclockwise winding) determines the direction of the triangle's normal vector. This is consistent with the `.obj` format for triangle meshes and is the most common convention.

The abbreviation `F` for triangle indices stands for "Face", to distinguish it from "Tetrahedron".

<p align="center">
    <img src="../media/triangle_orient.png" alt="Triangle Normal" width="50%" />
</p>

The direction of the triangle normal directly affects the application of Green's theorem. If the imported model produces unexpected simulation results, you may need to check whether the input triangle mesh is correct.

Additionally, `libuipc` allows manual specification of triangle normal directions using the `orient:<IndexT>` attribute on triangles. You can invert the normal of a triangle by setting its `orient` value to `-1`.

```python
from uipc import view
from uipc import builtin
orient = geo.triangles().find(builtin.orient)
view(orient)[:] = -1  # invert all normal directions
```

### Tetrahedron Orientation

In `libuipc`, tetrahedral meshes(tetmesh) can describe affine bodies and soft bodies.

The volume of a tetrahedron is determined by its topology index (`topo:<Vector4i>` attribute on tetrahedra), the formula is as follows:

$$
V=\frac{1}{6}(T_1-T_0)\times(T_2-T_0)\cdot(T_3-T_0)
$$

The surface direction of a positive volume tetrahedron is always outward.

<p align="center">
    <img src="../media/tetrahedron_orient.png" alt="Tetrahedron Orientation" width="50%" />
</p>

