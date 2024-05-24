---
title: SimplicialComplexAttributes
description: A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex. 
generator: doxide
---


# SimplicialComplexAttributes

**template &lt;typename SimplexSlot&gt; requires std::is_base_of_v&lt;ISimplexSlot, SimplexSlot&gt; class SimplicialComplexAttributes**



A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.
 




## Functions

| Name | Description |
| ---- | ----------- |
| [resize](#resize) |  :material-eye-outline: **See** :    [AttributeCollection::resize()](../AttributeCollection/#resize)  |
| [reserve](#reserve) |  :material-eye-outline: **See** :    [AttributeCollection::reserve()](../AttributeCollection/#reserve)  |
| [clear](#clear) |  :material-eye-outline: **See** :    [AttributeCollection::clear()](../AttributeCollection/#clear)  |
| [size](#size) |  :material-eye-outline: **See** :    [AttributeCollection::size()](../AttributeCollection/#size)  |
| [destroy](#destroy) |  :material-eye-outline: **See** :    [AttributeCollection::destroy()](../AttributeCollection/#destroy)  |
| [topo_view](#topo_view) | Get a non-const view of the topology. |
| [topo_view](#topo_view) | Get a const view of the topology, this function guarantees no data clone.  |
| [topo_is_shared](#topo_is_shared) | Query if the topology is owned by current simplicial complex.  |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |

## Function Details

### clear<a name="clear"></a>
!!! function "void                clear()"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::clear()](../AttributeCollection/#clear)
    
    

### destroy<a name="destroy"></a>
!!! function "void destroy(std::string_view name)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::destroy()](../AttributeCollection/#destroy)
    
    

### find<a name="find"></a>
!!! function "template &lt;typename T&gt; [[nodiscard]] auto find(std::string_view name)"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
         
    
    
    

### reserve<a name="reserve"></a>
!!! function "void                reserve(size_t size)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::reserve()](../AttributeCollection/#reserve)
    
    

### resize<a name="resize"></a>
!!! function "void                resize(size_t size)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::resize()](../AttributeCollection/#resize)
    
    

### size<a name="size"></a>
!!! function "[[nodiscard]] SizeT size() const"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::size()](../AttributeCollection/#size)
    
    

### topo_is_shared<a name="topo_is_shared"></a>
!!! function "[[nodiscard]] bool topo_is_shared() const"

    
    
    Query if the topology is owned by current simplicial complex.
         
    
    
    

### topo_view<a name="topo_view"></a>
!!! function "[[nodiscard]] auto topo_view()"

    
    
    Get a non-const view of the topology.
    
    !!! warning
         This function may cause a data clone if the topology is shared.
        
    

!!! function "[[nodiscard]] auto topo_view() const"

    
    
    Get a const view of the topology, this function guarantees no data clone.
         
    
    
    

