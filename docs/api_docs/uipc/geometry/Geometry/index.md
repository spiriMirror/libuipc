---
title: Geometry
description: A base geometry class that contains the instance attributes and the meta attributes. 
generator: doxide
---


# Geometry

**class Geometry : public IGeometry**



A base geometry class that contains the instance attributes and the meta attributes.
 




## Types

| Name | Description |
| ---- | ----------- |
| [MetaAttributes](MetaAttributes/index.md) | A wrapper class for the meta attributes of a geometry.  |
| [InstanceAttributes](InstanceAttributes/index.md) | A wrapper class for the instance attributes of a geometry.  |

## Functions

| Name | Description |
| ---- | ----------- |
| [transforms](#transforms) | A short-cut to get the non-const transforms attribute slot. |
| [transforms](#transforms) | A short-cut to get the const transforms attribute slot. |
| [meta](#meta) | Get the meta attributes of the geometry. |
| [instances](#instances) | Get the instance attributes of the geometry. |

## Function Details

### instances<a name="instances"></a>
!!! function "[[nodiscard]] InstanceAttributes instances()"

    
    
    Get the instance attributes of the geometry.
    
    :material-keyboard-return: **Return**
    :     The instance attributes of the geometry.
    
    

### meta<a name="meta"></a>
!!! function "[[nodiscard]] MetaAttributes meta()"

    
    
    Get the meta attributes of the geometry.
    
    :material-keyboard-return: **Return**
    :    The meta attributes of the geometry.
    
    

### transforms<a name="transforms"></a>
!!! function "[[nodiscard]] AttributeSlot&lt;Matrix4x4&gt;&amp; transforms()"

    
    
    A short-cut to get the non-const transforms attribute slot.
    
    :material-keyboard-return: **Return**
    :    The attribute slot of the non-const transforms.
    
    

!!! function "[[nodiscard]] const AttributeSlot&lt;Matrix4x4&gt;&amp; transforms() const"

    
    
    A short-cut to get the const transforms attribute slot.
    
    :material-keyboard-return: **Return**
    :    The attribute slot of the const transforms.
    
    

