---
title: VertexSlot
description: A slot for vertices in an abstract simplicial complex. 
generator: doxide
---


# VertexSlot

**class VertexSlot : public ISimplexSlot**



A slot for vertices in an abstract simplicial complex.
 




## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get a non-const view of the vertices. |
| [view](#view) | Get a const view of the vertices. |

## Function Details

### view<a name="view"></a>
!!! function "span&lt;IndexT&gt; view(VertexSlot&amp; slot)"

    
    
    Get a non-const view of the vertices.
    
    :material-location-enter: **Parameter** `slot`
    :    the slot for vertices
    
    :material-keyboard-return: **Return**
    :    A non-const view of the vertices
    
    

!!! function "span&lt;const IndexT&gt; view() const"

    
    
    Get a const view of the vertices.
    
    :material-keyboard-return: **Return**
    :    A const view of the vertices
    
    

