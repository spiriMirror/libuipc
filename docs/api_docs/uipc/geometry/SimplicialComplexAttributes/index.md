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
| [topo_is_shared](#topo_is_shared) | Query if the topology is owned by current simplicial complex.  |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |

## Function Details

### find<a name="find"></a>
!!! function "template &lt;typename T&gt; [[nodiscard]] auto find(std::string_view name)"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
         
    
    
    

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
         
    
    
    

