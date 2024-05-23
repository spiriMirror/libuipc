---
title: geometry
description: 
generator: doxide
---


# geometry



## Types

| Name | Description |
| ---- | ----------- |
| [SimplicialComplex](SimplicialComplex/index.md) | A simplicial complex is a collection of simplices, for example, vertices, edges, triangles, and tetrahedra. |
| [SimplicialComplexAttributes](SimplicialComplexAttributes/index.md) | A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.  |

## Functions

| Name | Description |
| ---- | ----------- |
| [linemesh](#linemesh) | Create a simplicial complex from a line mesh.  |
| [pointcloud](#pointcloud) | Create a simplicial complex from a point cloud.  |
| [tetmesh](#tetmesh) | Create a simplicial complex from a tetrahedral mesh.  |
| [trimesh](#trimesh) | Create a simplicial complex from a triangular mesh.  |

## Function Details

### linemesh<a name="linemesh"></a>
!!! function "SimplicialComplex linemesh(std::span&lt;const Vector3&gt; Vs, std::span&lt;const Vector2i&gt; Es)"

    
    
    Create a simplicial complex from a line mesh.
     
    
    
    

### pointcloud<a name="pointcloud"></a>
!!! function "SimplicialComplex pointcloud(std::span&lt;const Vector3&gt; Vs)"

    
    
    Create a simplicial complex from a point cloud.
     
    
    
    

### tetmesh<a name="tetmesh"></a>
!!! function "SimplicialComplex tetmesh(std::span&lt;const Vector3&gt; Vs, std::span&lt;const Vector4i&gt; Ts)"

    
    
    Create a simplicial complex from a tetrahedral mesh.
     
    
    
    

### trimesh<a name="trimesh"></a>
!!! function "SimplicialComplex trimesh(std::span&lt;const Vector3&gt; Vs, std::span&lt;const Vector3i&gt; Fs)"

    
    
    Create a simplicial complex from a triangular mesh.
     
    
    
    

