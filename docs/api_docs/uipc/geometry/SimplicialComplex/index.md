---
title: SimplicialComplex
description: A simplicial complex is a collection of simplices.
generator: doxide
---


# SimplicialComplex

**class  SimplicialComplex : public Geometry**



A simplicial complex is a collection of simplices.

In $\mathbb{R}^3$, a simplicial complex is defined as follows:
$$
K = (V, E, F, T),
$$
where $V$ is the set of vertices, $E$ is the set of edges, $F$ is the set of triangles, and $T$ is the set of tetrahedra.

:material-eye-outline: **See**
:    [Tutorial/Geometries](../../../../tutorial/geometries.md)



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
!!! function "[[nodiscard]] IndexT dim() const noexcept"

    
    
    Get the dimension of the simplicial complex.
    
    Return the maximum dimension of the simplices in the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    IndexT
    
    

### edges<a name="edges"></a>
!!! function "[[nodiscard]] EdgeAttributes  edges() noexcept"

    
    
    A wrapper of the edges and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    EdgeAttributes
    
    

### positions<a name="positions"></a>
!!! function "[[nodiscard]] AttributeSlot&lt;Vector3&gt;&amp; positions() noexcept"

    
    
    Get the positions of the vertices
    
    :material-keyboard-return: **Return**
    :    AttributeSlot<Vector3>&
    
    

!!! function "[[nodiscard]] const AttributeSlot&lt;Vector3&gt;&amp; positions() const noexcept"

    
    
    A short cut to get the positions of the vertices
    
    :material-keyboard-return: **Return**
    :    const AttributeSlot<Vector3>&
    
    

### tetrahedra<a name="tetrahedra"></a>
!!! function "[[nodiscard]] TetrahedronAttributes  tetrahedra() noexcept"

    
    
    A wrapper of the tetrahedra and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    TetrahedronAttributes
    
    

### triangles<a name="triangles"></a>
!!! function "[[nodiscard]] TriangleAttributes  triangles() noexcept"

    
    
    A wrapper of the triangles and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    TriangleAttributes
    
    

### vertices<a name="vertices"></a>
!!! function "[[nodiscard]] VertexAttributes  vertices() noexcept"

    
    
    A wrapper of the vertices and its attributes of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    VertexAttributes
    
    

