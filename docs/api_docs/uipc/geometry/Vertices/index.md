---
title: Vertices
description: A collection of vertices.
generator: doxide
---


# Vertices

**class Vertices final : public ISimplices**



A collection of vertices.

$V=\{0,1,2,...,N-1\}$, where $N$ is the number of vertices.
Normally, we don't store the vertice indices, because the indices is just iota $[0, N)$.



## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get the const view of the vertices, this method generates no data clone.  |
| [view](#view) | Get the non-const view of the vertices, this method may potentially generate data clone.  |

## Function Details

### view<a name="view"></a>
!!! function "[[nodiscard]] span&lt;const IndexT&gt; view() const"

    
    
    Get the const view of the vertices, this method generates no data clone.
         
    
    
    

!!! function "[[nodiscard]] span&lt;IndexT&gt; view()"

    
    
    Get the non-const view of the vertices, this method may potentially generate data clone.
         
    
    
    

