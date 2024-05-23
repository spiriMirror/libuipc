---
title: geometry
description: 
generator: doxide
---


# geometry



## Types

| Name | Description |
| ---- | ----------- |
| [Edges](Edges/index.md) | A collection of edges, $E=\{(i,j) \mid i,j\in V, i\neq j\}$, where $V$ is the set of vertices.  |
| [IAttribute](IAttribute/index.md) | An abstract class to represent a geometry attribute.  |
| [IGeometry](IGeometry/index.md) | An abstract class for geometry  |
| [ISimplices](ISimplices/index.md) | An abstract class for simplices, special type of topological elements. |
| [ITopoElements](ITopoElements/index.md) | An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements.  |
| [Simplices](Simplices/index.md) | General class to represent simplices, typically used for edges, triangles, tetrahedra. |
| [SimplicialComplex](SimplicialComplex/index.md) | A simplicial complex is a collection of simplices, for example, vertices, edges, triangles, and tetrahedra. |
| [SimplicialComplexAttributes](SimplicialComplexAttributes/index.md) | A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.  |
| [Tetrahedra](Tetrahedra/index.md) | A collection of tetrahedra, $T=\{(i,j,k,l) \mid i,j,k,l\in V, i\neq j\neq k\neq l\}$, where $V$ is the set of vertices.  |
| [Triangles](Triangles/index.md) | A collection of triangles, $F=\{(i,j,k) \mid i,j,k\in V, i\neq j\neq k\}$, where $V$ is the set of vertices.  |
| [Vertices](Vertices/index.md) | A collection of vertices, $V=\{0,1,2,...,N-1\}$ |

## Functions

| Name | Description |
| ---- | ----------- |
| [linemesh](#linemesh) | Create a simplicial complex from a line mesh.  |
| [pointcloud](#pointcloud) | Create a simplicial complex from a point cloud.  |
| [tetmesh](#tetmesh) | Create a simplicial complex from a tetrahedral mesh.  |
| [trimesh](#trimesh) | Create a simplicial complex from a triangular mesh.  |

## Function Details

### linemesh<a name="linemesh"></a>
!!! function "[[nodiscard]] SimplicialComplex linemesh(std::span&lt;const Vector3&gt;  Vs, std::span&lt;const Vector2i&gt; Es)"

    
    
    Create a simplicial complex from a line mesh.
     
    
    
    

### pointcloud<a name="pointcloud"></a>
!!! function "[[nodiscard]] SimplicialComplex pointcloud(std::span&lt;const Vector3&gt; Vs)"

    
    
    Create a simplicial complex from a point cloud.
     
    
    
    

### tetmesh<a name="tetmesh"></a>
!!! function "[[nodiscard]] SimplicialComplex tetmesh(std::span&lt;const Vector3&gt;  Vs, std::span&lt;const Vector4i&gt; Ts)"

    
    
    Create a simplicial complex from a tetrahedral mesh.
     
    
    
    

### trimesh<a name="trimesh"></a>
!!! function "[[nodiscard]] SimplicialComplex trimesh(std::span&lt;const Vector3&gt;  Vs, std::span&lt;const Vector3i&gt; Fs)"

    
    
    Create a simplicial complex from a triangular mesh.
     
    
    
    

