---
title: SimplicialComplex
description: A simplicial complex is a collection of simplices.
generator: doxide
---


# SimplicialComplex

**class SimplicialComplex : public Geometry**



A simplicial complex is a collection of simplices.

In $\mathbb{R}^3$, a simplicial complex is a collection of vertices, edges, triangles, and tetrahedra, such that
$$
K = (V, E, F, T),
$$
where $V$ is the set of vertices, $E$ is the set of edges,
$F$ is the set of triangles, and $T$ is the set of tetrahedra.

In addition to the topology, a simplicial complex have at least one attribute called "position",
representing the positions of the vertices.

Because "position" is so common, we provide a short cut to get the positions of the vertices, as shown below:

```cpp
auto mesh = geometry::tetmesh(Vs, Ts);
auto& attr_pos = mesh.positions();
```

!!! note
     To use `geometry::tetmesh`, you need to `#include <uipc/geometry/factory.h>`.

:material-eye-outline: **See**
:    [geometry::tetmesh()](../index.md#tetmesh)


We can also use the generic API to get the positions(or any other attributes) of the vertices, as shown below:
```cpp
auto VA = mesh.vertices();
auto pos = VA.find<Vector3>("position");
```

To the underlying attributes of the simplicial complex, we need to create a view of the attributes, as shown below:
```cpp
// const view
std::as_const(pos)->view();
// non-const view
pos->view();
```

!!! tip
     A non-const view of the attribute may cause data clone if the attribute is shared.
    If you don't tend to modify the attribute, always use the const version of the view.

!!! danger
     Never store a view of any attribute, because the view may become invalid after the attribute is modified.
    Always create a new view when you need it. Don't mind, the view is lightweight. :white_check_mark:

To get the tetrahedra of the simplicial complex, we can use the following code:
```cpp
auto TA  = mesh.tetrahedra();
```
You may want to add new attributes to the tetrahedra, for example,
in [Continuum Mechanics](https://en.wikipedia.org/wiki/Continuum_mechanics),
the deformation gradient $\mathbf{F} \in \mathbb{R}^{3\times 3}$ is an important attribute of the tetrahedra.
We can add the attribute to the tetrahedra as shown below:
```cpp
auto& F = TA.create<Matrix3x3>("F");
auto F_view = F.view();
std::fill(F_view.begin(), F_view.end(), Matrix3x3::Identity());
```

:material-eye-outline: **See**
:    [Tutorial/Geometry](../../../../tutorial/geometry.md)



## Types

| Name | Description |
| ---- | ----------- |
| [VertexAttributes](VertexAttributes/index.md) | Alias for the vertex attributes |
| [EdgeAttributes](EdgeAttributes/index.md) | Alias for the edge attributes |
| [TriangleAttributes](TriangleAttributes/index.md) | Alias for the triangle attributes |
| [TetrahedronAttributes](TetrahedronAttributes/index.md) | Alias for the tetrahedron attributes |

## Functions

| Name | Description |
| ---- | ----------- |
| [positions](#positions) | Get the positions of the vertices |
| [positions](#positions) | A short cut to get the positions of the vertices |
| [vertices](#vertices) | A wrapper of the vertices and its attributes of the simplicial complex. |
| [edges](#edges) | A wrapper of the edges and its attributes of the simplicial complex. |
| [triangles](#triangles) | A wrapper of the triangles and its attributes of the simplicial complex. |
| [tetrahedra](#tetrahedra) | A wrapper of the tetrahedra and its attributes of the simplicial complex. |
| [dim](#dim) | Get the dimension of the simplicial complex. |

## Function Details

### dim<a name="dim"></a>
!!! function "[[nodiscard]] IndexT dim() const"

    
    
    Get the dimension of the simplicial complex.
    
    Return the maximum dimension of the simplices in the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    IndexT
    
    

### edges<a name="edges"></a>
!!! function "[[nodiscard]] EdgeAttributes edges()"

    
    
    A wrapper of the edges and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    EdgeAttributes
    
    

### positions<a name="positions"></a>
!!! function "[[nodiscard]] AttributeSlot&lt;Vector3&gt;&amp; positions()"

    
    
    Get the positions of the vertices
    
    :material-keyboard-return: **Return**
    :    AttributeSlot<Vector3>&
    
    

!!! function "[[nodiscard]] const AttributeSlot&lt;Vector3&gt;&amp; positions() const"

    
    
    A short cut to get the positions of the vertices
    
    :material-keyboard-return: **Return**
    :    const AttributeSlot<Vector3>&
    
    

### tetrahedra<a name="tetrahedra"></a>
!!! function "[[nodiscard]] TetrahedronAttributes tetrahedra()"

    
    
    A wrapper of the tetrahedra and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    TetrahedronAttributes
    
    

### triangles<a name="triangles"></a>
!!! function "[[nodiscard]] TriangleAttributes triangles()"

    
    
    A wrapper of the triangles and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    TriangleAttributes
    
    

### vertices<a name="vertices"></a>
!!! function "[[nodiscard]] VertexAttributes vertices()"

    
    
    A wrapper of the vertices and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    VertexAttributes
    
    

