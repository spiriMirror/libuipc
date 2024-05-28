---
title: SimplicialComplexAttributes
description: A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex. 
generator: doxide
---


# SimplicialComplexAttributes

**template &lt;typename SimplexSlotT&gt; requires std::is_base_of_v&lt;ISimplexSlot, SimplexSlotT&gt; class SimplicialComplexAttributes**



A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.
 




## Functions

| Name | Description |
| ---- | ----------- |
| [topo](#topo) | Get the topology of the simplicial complex. |
| [resize](#resize) |  :material-eye-outline: **See** :    [AttributeCollection::resize()](../AttributeCollection/#resize)  |
| [reserve](#reserve) |  :material-eye-outline: **See** :    [AttributeCollection::reserve()](../AttributeCollection/#reserve)  |
| [clear](#clear) |  :material-eye-outline: **See** :    [AttributeCollection::clear()](../AttributeCollection/#clear)  |
| [size](#size) |  :material-eye-outline: **See** :    [AttributeCollection::size()](../AttributeCollection/#size)  |
| [destroy](#destroy) |  :material-eye-outline: **See** :    [AttributeCollection::destroy()](../AttributeCollection/#destroy)  |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |

## Function Details

### clear<a name="clear"></a>
!!! function "void clear()"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::clear()](../AttributeCollection/#clear)
    
    

### destroy<a name="destroy"></a>
!!! function "void destroy(std::string_view name)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::destroy()](../AttributeCollection/#destroy)
    
    

### find<a name="find"></a>
!!! function "template &lt;typename T&gt; [[nodiscard]] decltype(auto) find(std::string_view name)"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
         
    
    
    

!!! function "template &lt;typename T&gt; [[nodiscard]] decltype(auto) find(std::string_view name) const"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
        
    
    
    

### reserve<a name="reserve"></a>
!!! function "void reserve(SizeT size)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::reserve()](../AttributeCollection/#reserve)
    
    

### resize<a name="resize"></a>
!!! function "void resize(SizeT size)"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::resize()](../AttributeCollection/#resize)
    
    

### size<a name="size"></a>
!!! function "[[nodiscard]] SizeT size() const noexcept"

    
    
    :material-eye-outline: **See**
    :    [AttributeCollection::size()](../AttributeCollection/#size)
    
    

### topo<a name="topo"></a>
!!! function "[[nodiscard]] Topo topo() noexcept"

    
    
    Get the topology of the simplicial complex.
    
    :material-keyboard-return: **Return**
    :    Topo
    
    

