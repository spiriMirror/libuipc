# Implicit Geometry UID

The Implicit Geometry UID is a unique identifier for an implicit geometry known by `libuipc`, which is a 64-bit unsigned integer.

The official implicit geometry UID has a range of $[0, 2^{32}-1]$. The range $[2^{32}, 2^{64}-1]$ is reserved for user-defined implicit geometries.

Every official implicit geometry will be documented in this specification. A user-defined implicit geometry can apply for an official implicit geometry UID by submitting a pull request to the `libuipc` repository, After code review, the implicit geometry will be added to the official implicit geometry list.

The related documentation of the implicit geometry will be added to the [Implicit Geometries/](./implicit_geometries/index.md) directory.

When creating a implicit geometry, the `implicit_geometry_uid` attribute of the `meta` attribute of the geometry will be set to the implicit geometry UID. The backend will use this UID to determine the implicit geometry of the geometry, and try to find the related coefficients from the attributes of the geometry (in `meta` or `instances`).