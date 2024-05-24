---
title: Topo
description: A wrapper of the topology of the simplicial complex. 
generator: doxide
---


# Topo

**class Topo**



A wrapper of the topology of the simplicial complex.
     




## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get a non-const view of the topology. |
| [view](#view) | Get a const view of the topology, this function guarantees no data clone.  |
| [is_shared](#is_shared) | Query if the topology is owned by current simplicial complex.  |

## Function Details

### is_shared<a name="is_shared"></a>
!!! function "[[nodiscard]] bool is_shared() const"

    
    
    Query if the topology is owned by current simplicial complex.
             
    
    
    

### view<a name="view"></a>
!!! function "[[nodiscard]] auto view()"

    
    
    Get a non-const view of the topology.
    
    !!! warning
         This function may cause a data clone if the topology is shared.
        
    

!!! function "[[nodiscard]] auto view() const"

    
    
    Get a const view of the topology, this function guarantees no data clone.
             
    
    
    

