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
| [Attribute](Attribute/index.md) | Template class to represent a geometries attribute of type T. |
| [AttributeCollection](AttributeCollection/index.md) | A collection of geometries attributes. |
| [AttributeSlot](AttributeSlot/index.md) | Template class to represent a geometries attribute slot of type T in a geometries attribute collection. |
| [EdgeSlot](EdgeSlot/index.md) | Alias for a slot for edges in an abstract simplicial complex.  |
| [Edges](Edges/index.md) | A collection of edges. |
| [Geometry](Geometry/index.md) | A base geometries class that contains the instance attributes and the meta attributes.  |
| [IAttribute](IAttribute/index.md) | An abstract class to represent a geometries attribute.  |
| [IAttributeSlot](IAttributeSlot/index.md) | An abstract class to represent a geometries attribute slot in a geometries attribute collection. |
| [IGeometry](IGeometry/index.md) | An abstract class for geometries  |
| [ISimplexSlot](ISimplexSlot/index.md) | An abstract class representing a simplex slot in an abstract simplicial complex.  |
| [ISimplices](ISimplices/index.md) | An abstract class for simplices, special type of topological elements. |
| [ITopoElements](ITopoElements/index.md) | An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements.  |
| [Simplices](Simplices/index.md) | General class to represent simplices, typically used for edges, triangles, tetrahedra. |
| [SimplicialComplex](SimplicialComplex/index.md) | A simplicial complex is a collection of simplices. |
| [SimplicialComplexAttributes](SimplicialComplexAttributes/index.md) | A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.  |
| [SimplicialComplexIO](SimplicialComplexIO/index.md) | A class for reading and writing simplicial complex.  |
| [SimplicialComplexTopo](SimplicialComplexTopo/index.md) | A wrapper of the topology of the simplicial complex.  |
| [Tetrahedra](Tetrahedra/index.md) | A collection of tetrahedra. |
| [TetrahedronSlot](TetrahedronSlot/index.md) | Alias for a slot for tetrahedra in an abstract simplicial complex.  |
| [TriangleSlot](TriangleSlot/index.md) | Alias for a slot for triangles in an abstract simplicial complex.  |
| [Triangles](Triangles/index.md) | A collection of triangles. |
| [VertexSlot](VertexSlot/index.md) | A slot for vertices in an abstract simplicial complex.  |
| [Vertices](Vertices/index.md) | A collection of vertices. |

## Functions

| Name | Description |
| ---- | ----------- |
| [backend_view](#backend_view) | An interface to get the backend buffer view  |
| [backend_view](#backend_view) | Get the backend view of the topology, this function guarantees no data clone.  |
| [facet_closure](#facet_closure) | Generate the closure from a collection of facet simplices, who only have the top dimension simplices. |
| [ground](#ground) | Create a gound plane. |
| [label_surface](#label_surface) | Label the surface of a simplicial complex. |
| [label_triangle_orient](#label_triangle_orient) | Label the orientation of the triangles in the simplicial complex. |
| [linemesh](#linemesh) | Create a simplicial complex from a line mesh. |
| [pointcloud](#pointcloud) | Create a simplicial complex from a point cloud. |
| [tetmesh](#tetmesh) | Create a simplicial complex from a tetrahedral mesh. |
| [trimesh](#trimesh) | Create a simplicial complex from a triangle mesh. |
| [view](#view) | An interface to create a non-const view  |
| [view](#view) | Get a non-const view of the topology, this function may clone the data.  |

## Function Details

### backend_view<a name="backend_view"></a>
!!! function "template &lt;typename T, typename U&gt; backend::BufferView backend_view(U&amp;)"

    
    
    An interface to get the backend buffer view
     
    
    
    

!!! function "template &lt;&gt; backend::BufferView backend_view( SimplicialComplexTopo&lt;const VertexSlot&gt;&amp;&amp; v) noexcept"

    
    
    Get the backend view of the topology, this function guarantees no data clone.
         
    
    
    

### facet_closure<a name="facet_closure"></a>
!!! function "[[nodiscard]] SimplicialComplex facet_closure(const SimplicialComplex&amp; complex)"

    
    
    Generate the closure from a collection of facet simplices, who only have the top dimension simplices.
    
    E.g.
    1) the input 3D simplicial complex (tetmesh) can only have tetrahedrons (no triangles, edges).
    2) the input 2D simplicial complex (trimesh) can only have triangles (no edges).
    3) the input 1D simplicial complex (linemesh) can only have edges.
    4) the input 0D simplicial complex (pointcloud) can only have vertices.
    
    

### ground<a name="ground"></a>
!!! function "[[nodiscard]] ImplicitGeometry ground(Float height = 0.0)"

    
    
    Create a gound plane.
    
    :material-location-enter: **Parameter** `height`
    :    The height of the ground plane
        
    

### label_surface<a name="label_surface"></a>
!!! function "[[nodiscard]] SimplicialComplex label_surface(const SimplicialComplex&amp; sc)"

    
    
    Label the surface of a simplicial complex.
    
    1) label 'is_surf':<IndexT> on vertices/edges/triangles/tetrahedra
    2) set  'parent_id':<IndexT> on triangles, indicating the parent tetrahedron
    
    

### label_triangle_orient<a name="label_triangle_orient"></a>
!!! function "[[nodiscard]] SimplicialComplex label_triangle_orient(const SimplicialComplex&amp; sc)"

    
    
    Label the orientation of the triangles in the simplicial complex.
    
    Set 'orient':<Index> for each triangle in the simplicial complex.
    1) orient=1 means the triangle is oriented outward the tetrahedron.
    2) orient=0 means the orientation is undetermined.
    3) orient=-1 means the triangle is oriented inward the tetrahedron.
    
    

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
    
    

### view<a name="view"></a>
!!! function "template &lt;typename T, typename U&gt; span&lt;T&gt; view(U&amp;)"

    
    
    An interface to create a non-const view
     
    
    
    

!!! function "template &lt;&gt; span&lt;typename VertexSlot::ValueT&gt; view(SimplicialComplexTopo&lt;VertexSlot&gt;&amp;&amp; v)"

    
    
    Get a non-const view of the topology, this function may clone the data.
         
    
    
    

