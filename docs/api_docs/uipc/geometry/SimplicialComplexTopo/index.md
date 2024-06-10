---
title: SimplicialComplexTopo
description: A wrapper of the topology of the simplicial complex. 
generator: doxide
---


# SimplicialComplexTopo

**template &lt;std::derived_from&lt;ISimplexSlot&gt; SimplexSlotT&gt; class SimplicialComplexTopo**



A wrapper of the topology of the simplicial complex.





## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get a non-const view of the topology, this function may clone the data.  |
| [backend_view](#backend_view) | Get the backend view of the topology, this function guarantees no data clone.  |
| [view](#view) | Get a const view of the topology, this function guarantees no data clone.  |
| [is_shared](#is_shared) | Query if the topology is owned by current simplicial complex.  |

## Function Details

### backend_view<a name="backend_view"></a>
!!! function "backend::BufferView backend_view(SimplicialComplexTopo&lt;SimplexSlot&lt;N&gt;&gt;&amp;&amp; v) noexcept"

    
    
    Get the backend view of the topology, this function guarantees no data clone.
         
    
    
    

### is_shared<a name="is_shared"></a>
!!! function "[[nodiscard]] bool is_shared() &amp;&amp; noexcept"

    
    
    Query if the topology is owned by current simplicial complex.
         
    
    
    

### view<a name="view"></a>
!!! function "span&lt;typename SimplexSlot&lt;N&gt;::ValueT&gt; view(SimplicialComplexTopo&lt;SimplexSlot&lt;N&gt;&gt;&amp;&amp; v)"

    
    
    Get a non-const view of the topology, this function may clone the data.
         
    
    
    

!!! function "[[nodiscard]] auto view() &amp;&amp; noexcept"

    
    
    Get a const view of the topology, this function guarantees no data clone.
         
    
    
    

