---
title: AbstractSimplicialComplex
description: Represents an abstract simplicial complex, containing vertices, edges, triangles, and tetrahedra.
generator: doxide
---


# AbstractSimplicialComplex

**class AbstractSimplicialComplex**



Represents an abstract simplicial complex, containing vertices, edges, triangles, and tetrahedra.

!!! note
     Abstract simplicial complex does not contain any geometric information, such as coordinates of vertices.
    


## Functions

| Name | Description |
| ---- | ----------- |
| [vertices](#vertices) | Get the non-const slot for vertices. |
| [vertices](#vertices) | Get the const slot for vertices. |
| [edges](#edges) | Get the non-const slot for edges. |
| [edges](#edges) | Get the const slot for edges. |
| [triangles](#triangles) | Get the non-const slot for triangles. |
| [triangles](#triangles) | Get the const slot for triangles. |
| [tetrahedra](#tetrahedra) | Get the non-const slot for tetrahedra. |
| [tetrahedra](#tetrahedra) | Get the const slot for tetrahedra. |

## Function Details

### edges<a name="edges"></a>
!!! function "EdgeSlot&amp; edges()"

    
    
    Get the non-const slot for edges.
    
    :material-keyboard-return: **Return**
    :    a non-const slot for edges
    
    

!!! function "const EdgeSlot&amp; edges() const"

    
    
    Get the const slot for edges.
    
    :material-keyboard-return: **Return**
    :    a const slot for edges
    
    

### tetrahedra<a name="tetrahedra"></a>
!!! function "TetrahedronSlot&amp; tetrahedra()"

    
    
    Get the non-const slot for tetrahedra.
    
    :material-keyboard-return: **Return**
    :    a non-const slot for tetrahedra
    
    

!!! function "const TetrahedronSlot&amp; tetrahedra() const"

    
    
    Get the const slot for tetrahedra.
    
    :material-keyboard-return: **Return**
    :    a const slot for tetrahedra
    
    

### triangles<a name="triangles"></a>
!!! function "TriangleSlot&amp; triangles()"

    
    
    Get the non-const slot for triangles.
    
    :material-keyboard-return: **Return**
    :    a non-const slot for triangles
    
    

!!! function "const TriangleSlot&amp; triangles() const"

    
    
    Get the const slot for triangles.
    
    :material-keyboard-return: **Return**
    :    a const slot for triangles
    
    

### vertices<a name="vertices"></a>
!!! function "VertexSlot&amp; vertices()"

    
    
    Get the non-const slot for vertices.
    
    :material-keyboard-return: **Return**
    :    a non-const slot for vertices
    
    

!!! function "const VertexSlot&amp; vertices() const"

    
    
    Get the const slot for vertices.
    
    :material-keyboard-return: **Return**
    :    a const slot for vertices
    
    

