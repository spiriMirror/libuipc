---
title: builtin
description: 
generator: doxide
---


# builtin



## Variables

| Name | Description |
| ---- | ----------- |
| [constitution](#constitution) | `constitution` attribute on geometries.meta()  |
| [constitution_uid](#constitution_uid) | `constitution_uid` attribute on geometries.meta(), uid is a unique identifier for a constitution * which is defined in the libuipc specification.  |
| [contact_element_id](#contact_element_id) | `contact_element_id` attribute on geometries.meta()  |
| [implicit_geometry_uid](#implicit_geometry_uid) | `implicit_geometry_uid` attribute on geometries.meta(), uid is a unique identifier for an implicit geometry * which is defined in the libuipc specification.  |
| [is_surf](#is_surf) | `is_surf` attribute on vertices/edges/triangles/tetrahedra... to indicate if the element is a surface element.  |
| [orient](#orient) | `orient` (value=-1,0,1) attribute on triangles to indicate the orientation of the triangle. |
| [parent_id](#parent_id) | `parent_id` attribute, indicates the parent simplex id  |
| [position](#position) | `position` attribute on geometries.vertices();  |
| [transform](#transform) | `transform` attribute on geometries.instances()  |

## Variable Details

### constitution<a name="constitution"></a>

!!! variable "constexpr std::string_view constitution"

    
    
    `constitution` attribute on geometries.meta()
     
    
    
    

### constitution_uid<a name="constitution_uid"></a>

!!! variable "constexpr std::string_view constitution_uid"

    
    
    `constitution_uid` attribute on geometries.meta(), uid is a unique identifier for a constitution
     * which is defined in the libuipc specification.
     
    
    
    

### contact_element_id<a name="contact_element_id"></a>

!!! variable "constexpr std::string_view contact_element_id"

    
    
    `contact_element_id` attribute on geometries.meta()
     
    
    
    

### implicit_geometry_uid<a name="implicit_geometry_uid"></a>

!!! variable "constexpr std::string_view implicit_geometry_uid"

    
    
    `implicit_geometry_uid` attribute on geometries.meta(), uid is a unique identifier for an implicit geometry
     * which is defined in the libuipc specification.
     
    
    
    

### is_surf<a name="is_surf"></a>

!!! variable "constexpr std::string_view is_surf"

    
    
    `is_surf` attribute on vertices/edges/triangles/tetrahedra... to indicate if the element is a surface element.
     
    
    
    

### orient<a name="orient"></a>

!!! variable "constexpr std::string_view orient"

    
    
    `orient` (value=-1,0,1) attribute on triangles to indicate the orientation of the triangle.
    
    1) 0 is the default value, which means the orientation is not determined, or the triangle is not a surface triangle.
    2) 1 means outward the tetrahedron
    3) -1 means inward the tetrahedron.
    
    

### parent_id<a name="parent_id"></a>

!!! variable "constexpr std::string_view parent_id"

    
    .`parent_id` attribute, indicates the parent simplex id 
     
    
    
    

### position<a name="position"></a>

!!! variable "constexpr std::string_view position"

    
    
    `position` attribute on geometries.vertices();
     
    
    
    

### transform<a name="transform"></a>

!!! variable "constexpr std::string_view transform"

    
    
    `transform` attribute on geometries.instances()
     
    
    
    

