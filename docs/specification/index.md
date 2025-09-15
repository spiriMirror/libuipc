# Libuipc Specification

This document, referred to as the "Libuipc Specification" defines the behavior of the library.

[TOC]

## Coordinates

The `libuipc` uses 3D coordinates to represent points in space. The coordinates are represented as a tuple of three real numbers `(x, y, z)` with units in meters. The convention used is the right-handed coordinate system, with the x-axis pointing to the right, the y-axis pointing up, and the z-axis pointing backwards.

![Coordinate System](../media/coordinates.png)

Of course you can take the Z-axis as the Up-axis, you only need to change the gravity direction in the scene configuration.
