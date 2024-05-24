---
title: InstanceAttributes
description: A wrapper class for the instance attributes of a geometry. 
generator: doxide
---


# InstanceAttributes

**class InstanceAttributes**



A wrapper class for the instance attributes of a geometry.
     




## Functions

| Name | Description |
| ---- | ----------- |
| [resize](#resize) |  :material-eye-outline: **See** :    [AttributeCollection::resize()](../../AttributeCollection/#resize)  |
| [reserve](#reserve) |  :material-eye-outline: **See** :    [AttributeCollection::reserve()](../../AttributeCollection/#reserve)  |
| [clear](#clear) |  :material-eye-outline: **See** :    [AttributeCollection::clear()](../../AttributeCollection/#clear)  |
| [size](#size) |  :material-eye-outline: **See** :    [AttributeCollection::size()](../../AttributeCollection/#size)  |
| [destroy](#destroy) |  :material-eye-outline: **See** :    [AttributeCollection::destroy()](../../AttributeCollection/#destroy)  |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |
| [create](#create) | Create an attribute with the given name.  |

## Function Details

### clear<a name="clear"></a>
!!! function "void clear()"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::clear()](../../AttributeCollection/#clear)
    
    

### create<a name="create"></a>
!!! function "template &lt;typename T&gt; decltype(auto) create(std::string_view name, const T&amp; init_value = {})"

    
    
    Create an attribute with the given name.
             
    
    
    

### destroy<a name="destroy"></a>
!!! function "void destroy(std::string_view name)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::destroy()](../../AttributeCollection/#destroy)
    
    

### find<a name="find"></a>
!!! function "template &lt;typename T&gt; [[nodiscard]] auto find(std::string_view name)"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
             
    
    
    

### reserve<a name="reserve"></a>
!!! function "void reserve(size_t size)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::reserve()](../../AttributeCollection/#reserve)
    
    

### resize<a name="resize"></a>
!!! function "void resize(size_t size)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::resize()](../../AttributeCollection/#resize)
    
    

### size<a name="size"></a>
!!! function "[[nodiscard]] SizeT size() const"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::size()](../../AttributeCollection/#size)
    
    

