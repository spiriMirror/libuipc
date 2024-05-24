---
title: VertexSlot
description: A slot for vertices in an abstract simplicial complex. 
generator: doxide
---


# VertexSlot

**class VertexSlot : public ISimplexSlot**



A slot for vertices in an abstract simplicial complex.
 




## Operators

| Name | Description |
| ---- | ----------- |
| [operator->](#operator_u002d_u003e) | Get the non-const underlying vertices, this method may potentially generate data clone. |
| [operator->](#operator_u002d_u003e) | Get the const underlying vertices, this method generates no data clone. |

## Operator Details

### operator-><a name="operator_u002d_u003e"></a>

!!! function "Vertices&#42;       operator-&gt;()"

    
    
    Get the non-const underlying vertices, this method may potentially generate data clone.
    
    :material-keyboard-return: **Return**
    :    non-const underlying vertices
    
    

!!! function "const Vertices&#42; operator-&gt;() const"

    
    
    Get the const underlying vertices, this method generates no data clone.
    
    :material-keyboard-return: **Return**
    :    const underlying vertices
    
    

