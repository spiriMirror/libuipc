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

## Function Details

### positions<a name="positions"></a>
!!! function "AttributeSlot&lt;Vector3&gt;&amp; positions()"

    
    
    A short cut to get the positions of the vertices
         
    
    
    

!!! function "const AttributeSlot&lt;Vector3&gt;&amp; positions() const"

    
    
    A short cut to get the positions of the vertices
         
    
    
    

