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
| [topo_view](#topo_view) | Get a non-const view of the topology. |
| [topo_view](#topo_view) | Get a const view of the topology, this function guarantees no data clone.  |
| [topo_is_owned](#topo_is_owned) | Query if the topology is owned by current simplicial complex.  |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |

## Function Details

### find<a name="find"></a>
!!! function "template &lt;typename T&gt; auto find(std::string_view name)"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
         
    
    
    

### topo_is_owned<a name="topo_is_owned"></a>
!!! function "bool topo_is_owned() const"

    
    
    Query if the topology is owned by current simplicial complex.
         
    
    
    

### topo_view<a name="topo_view"></a>
!!! function "auto topo_view()"

    
    
    Get a non-const view of the topology.
    
    !!! warning
         This function may make a data clone if the topology is shared. if you want to avoid data clone, use `std::as_const(this_instance).topo_view()` instead.
        
    

!!! function "auto topo_view() const"

    
    
    Get a const view of the topology, this function guarantees no data clone.
         
    
    
    

