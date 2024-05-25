---
title: SimplicialComplexTopo
description: A wrapper of the topology of the simplicial complex. 
generator: doxide
---


# SimplicialComplexTopo

**template &lt;typename SimplexSlotT&gt; requires std::is_base_of_v&lt;ISimplexSlot, SimplexSlotT&gt; class SimplicialComplexTopo**



A wrapper of the topology of the simplicial complex.





## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get a const view of the topology, this function guarantees no data clone.  |
| [is_shared](#is_shared) | Query if the topology is owned by current simplicial complex.  |

## Function Details

### is_shared<a name="is_shared"></a>
!!! function "[[nodiscard]] bool is_shared() &amp;&amp;"

    
    
    Query if the topology is owned by current simplicial complex.
         
    
    
    

### view<a name="view"></a>
!!! function "[[nodiscard]] auto view() &amp;&amp;"

    
    
    Get a const view of the topology, this function guarantees no data clone.
         
    
    
    

