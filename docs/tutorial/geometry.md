# Geometry

## Clone on Write

Geometries in `libuipc` are implemented with the `Clone on Write` strategy. Any inital copy of a geometry is a shallow copy, which means, the data of the geometry is shared. Any creation of a non-const view of the geometry will trigger a minimal clone of the modified part of the geometry.

A simple example is shown below:
```cpp
auto foo = uipc::geometry::tetmesh(Vs,Ts);
auto bar = foo;
```
Here, `bar` is just a shallow copy of the `foo`.
No matter `bar` or `foo` is modified, the related internal part of the data will be cloned.

For example, we create a non-const view of the positions of the mesh `bar`:
```cpp
auto VA  = bar.vertices();
auto pos = VA.find<Vector3>("position");
pos->is_shared(); // true
auto non_const_view = pos->view();
pos->is_shared(); // false
```
Here, `pos->is_shared()` first return `true`, which means the position data is shared. After we create a non-const view of the position data, `pos->is_shared()` will return `false`, which means, `pos` is exclusively belong to `bar`.
```cpp
TA = bar.tetrahedra();
TA.topo_is_shared(); // true
```
Here, `TA.topo_is_shared()` will return `true`, because we don't modify the topology, so the topology of `foo` and `bar` is still shared.

Such a design minimizes the geometry memory usage.

!!!Warning
    Be careful when you create a view of the geometry. Always consider using `std::as_const(...)` to make sure that you call the const version of the function. A mistake of calling the non-const version will potentially trigger a clone of the geometry. Although the behavior is safe, it may cause a performance issue.

## Simplical Complex

Simplical complex is a general representation of an explicit mesh. In $\mathbb{R}^3$, a simplical complex can be a tetrahedral mesh, a triangle mesh, a line mesh, or a point cloud, which have a dimension of 3, 2, 1, or 0, respectively.

```cpp
auto tetmesh = uipc::geometry::tetmesh(Vs,Ts);
tetmesh.dim(); // 3
auto trimesh = uipc::geometry::trimesh(Vs,Fs);
trimesh.dim(); // 2
auto linemesh = uipc::geometry::linemesh(Vs,Es);
linemesh.dim(); // 1
auto pointcloud = uipc::geometry::pointcloud(Vs);
pointcloud.dim(); // 0
```

