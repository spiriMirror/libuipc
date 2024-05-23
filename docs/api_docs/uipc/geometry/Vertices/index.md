---
title: Vertices
description: A collection of vertices, $V=\{0,1,2,...,N-1\}$
generator: doxide
---


# Vertices

**class Vertices final : public ISimplices**



A collection of vertices, $V=\{0,1,2,...,N-1\}$

Normally, we don't store the vertice indices, because the indices is just iota $[0, N)$.




## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get the const view of the vertices, this method generates no data clone. |
| [view](#view) | Get the non-const view of the vertices, this method may potentially generate data clone. |

## Function Details

### view<a name="view"></a>
!!! function "[[nodiscard]] std::span&lt;const IndexT&gt; view() const"

    
    
    Get the const view of the vertices, this method generates no data clone.
    
    :material-keyboard-return: **Return**
    :    std::span<const IndexT>
    
    

!!! function "[[nodiscard]] std::span&lt;IndexT&gt;       view()"

    
    
    Get the non-const view of the vertices, this method may potentially generate data clone.
    
    :material-keyboard-return: **Return**
    :    std::span<IndexT>
    
    

