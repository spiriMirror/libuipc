---
title: geometry
description: 
generator: doxide
---


# geometry



## Types

| Name | Description |
| ---- | ----------- |
| [AbstractSimplicialComplex](AbstractSimplicialComplex/index.md) | Represents an abstract simplicial complex, containing vertices, edges, triangles, and tetrahedra. |
| [Attribute](Attribute/index.md) | Template class to represent a geometry attribute of type T. |
| [AttributeCollection](AttributeCollection/index.md) | A collection of geometry attributes. |
| [AttributeSlot](AttributeSlot/index.md) | Template class to represent a geometry attribute slot of type T in a geometry attribute collection. |
| [Edges](Edges/index.md) | A collection of edges. |
| [Geometry](Geometry/index.md) | A base geometry class that contains the instance attributes and the meta attributes.  |
| [IAttribute](IAttribute/index.md) | An abstract class to represent a geometry attribute.  |
| [IAttributeSlot](IAttributeSlot/index.md) | An abstract class to represent a geometry attribute slot in a geometry attribute collection. |
| [IGeometry](IGeometry/index.md) | An abstract class for geometry  |
| [ISimplexSlot](ISimplexSlot/index.md) | An abstract class representing a simplex slot in an abstract simplicial complex. |
| [ISimplices](ISimplices/index.md) | An abstract class for simplices, special type of topological elements. |
| [ITopoElements](ITopoElements/index.md) | An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements.  |
| [Simplices](Simplices/index.md) | General class to represent simplices, typically used for edges, triangles, tetrahedra. |
| [SimplicialComplex](SimplicialComplex/index.md) | A simplicial complex is a collection of simplices. |
| [SimplicialComplexAttributes](SimplicialComplexAttributes/index.md) | A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.  |
| [SimplicialComplexIO](SimplicialComplexIO/index.md) | A class for reading and writing simplicial complex.  |
| [Tetrahedra](Tetrahedra/index.md) | A collection of tetrahedra. |
| [Triangles](Triangles/index.md) | A collection of triangles. |
| [VertexSlot](VertexSlot/index.md) | A slot for vertices in an abstract simplicial complex.  |
| [Vertices](Vertices/index.md) | A collection of vertices. |

## Functions

| Name | Description |
| ---- | ----------- |
| [linemesh](#linemesh) | Create a simplicial complex from a line mesh. |
| [pointcloud](#pointcloud) | Create a simplicial complex from a point cloud. |
| [tetmesh](#tetmesh) | Create a simplicial complex from a tetrahedral mesh. |
| [trimesh](#trimesh) | Create a simplicial complex from a triangle mesh. |

## Function Details

### linemesh<a name="linemesh"></a>
!!! function "[[nodiscard]] SimplicialComplex linemesh(span&lt;const Vector3&gt;  Vs, span&lt;const Vector2i&gt; Es)"

    
    
    Create a simplicial complex from a line mesh.
    
    :material-location-enter: **Parameter** `Vs`
    :    The vertex positions of the line mesh
    
    :material-location-enter: **Parameter** `Es`
    :    The edges of the line mesh
    
    :material-keyboard-return: **Return**
    :    SimplicialComplex
    
    

### pointcloud<a name="pointcloud"></a>
!!! function "[[nodiscard]] SimplicialComplex pointcloud(span&lt;const Vector3&gt; Vs)"

    
    
    Create a simplicial complex from a point cloud.
    
    :material-location-enter: **Parameter** `Vs`
    :    The vertex positions of the point cloud
    
    :material-keyboard-return: **Return**
    :    SimplicialComplex
    
    

### tetmesh<a name="tetmesh"></a>
!!! function "[[nodiscard]] SimplicialComplex tetmesh(span&lt;const Vector3&gt;  Vs, span&lt;const Vector4i&gt; Ts)"

    
    
    Create a simplicial complex from a tetrahedral mesh.
    
    :material-location-enter: **Parameter** `Vs`
    :    The vertex positions of the tetrahedral mesh
    
    :material-location-enter: **Parameter** `Ts`
    :    The tetrahedra of the tetrahedral mesh
        
    

### trimesh<a name="trimesh"></a>
!!! function "[[nodiscard]] SimplicialComplex trimesh(span&lt;const Vector3&gt;  Vs, span&lt;const Vector3i&gt; Fs)"

    
    
    Create a simplicial complex from a triangle mesh.
    
    :material-location-enter: **Parameter** `Vs`
    :    The vertex positions of the triangle mesh
    
    :material-location-enter: **Parameter** `Fs`
    :    The triangles of the triangle mesh
    
    :material-keyboard-return: **Return**
    :    SimplicialComplex
    
    

