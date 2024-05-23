---
title: SimplicialComplex
description: A simplicial complex is a collection of simplices, for example, vertices, edges, triangles, and tetrahedra.
generator: doxide
---


# SimplicialComplex

**class SimplicialComplex : public IGeometry**



A simplicial complex is a collection of simplices, for example, vertices, edges, triangles, and tetrahedra.

$K = (V, E, F, T)$, where $V$ is the set of vertices, $E$ is the set of edges,
$F$ is the set of triangles, and $T$ is the set of tetrahedra.

A simple example:
```cpp
auto mesh = geometry::tetmesh(Vs, Ts);
auto VA  = mesh.vertices();
auto TA  = mesh.tetrahedra();
auto pos = VA.find<Vector3>("position");
auto const_view = std::as_const(pos)->view();
auto non_const_view = pos->view();
```

!!! warning
     An non-const view of the attribute may cause data clone if the attribute is shared.
    To avoid data clone, use `std::as_const(this_instance).attribute_view()` instead.




## Functions

| Name | Description |
| ---- | ----------- |
| [positions](#positions) | A short cut to get the positions of the vertices  |
| [positions](#positions) | A short cut to get the positions of the vertices  |
| [vertices](#vertices) |  :material-keyboard-return: **Return** :    A wrapper of the vertices and its attributes of the simplicial complex.  |
| [edges](#edges) |  :material-keyboard-return: **Return** :    A wrapper of the edges and its attributes of the simplicial complex. :material-eye-outline: **See** :    EdgeAttributes  |
| [triangles](#triangles) |  :material-keyboard-return: **Return** :    A wrapper of the triangles and its attributes of the simplicial complex.  |
| [tetrahedra](#tetrahedra) |  :material-keyboard-return: **Return** :    A wrapper of the tetrahedra and its attributes of the simplicial complex.  |
| [dim](#dim) | Get the dimension of the simplicial complex.  |

## Function Details

### dim<a name="dim"></a>
!!! function "IndexT dim() const"

    
    
    Get the dimension of the simplicial complex.
        
    
    
    

### edges<a name="edges"></a>
!!! function "EdgeAttributes edges()"

    
    
    :material-keyboard-return: **Return**
    :    A wrapper of the edges and its attributes of the simplicial complex.
    
    :material-eye-outline: **See**
    :    EdgeAttributes
    
    

### positions<a name="positions"></a>
!!! function "AttributeSlot&lt;Vector3&gt;&amp; positions()"

    
    
    A short cut to get the positions of the vertices
         
    
    
    

!!! function "const AttributeSlot&lt;Vector3&gt;&amp; positions() const"

    
    
    A short cut to get the positions of the vertices
         
    
    
    

### tetrahedra<a name="tetrahedra"></a>
!!! function "TetrahedronAttributes tetrahedra()"

    
    
    :material-keyboard-return: **Return**
    :    A wrapper of the tetrahedra and its attributes of the simplicial complex.
    
    

### triangles<a name="triangles"></a>
!!! function "TriangleAttributes triangles()"

    
    
    :material-keyboard-return: **Return**
    :    A wrapper of the triangles and its attributes of the simplicial complex.
    
    
    

### vertices<a name="vertices"></a>
!!! function "VertexAttributes vertices()"

    
    
    :material-keyboard-return: **Return**
    :    A wrapper of the vertices and its attributes of the simplicial complex.
    
    

