---
title: Simplices
description: General class to represent simplices, typically used for edges, triangles, tetrahedra.
generator: doxide
---


# Simplices

**template &lt;IndexT N&gt; class Simplices final : public ISimplices**



General class to represent simplices, typically used for edges, triangles, tetrahedra.

:material-code-tags: **Template parameter** `N`
:   
    


## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get the const view of the simplices, this method generates no data clone. |
| [view](#view) | Get the non-const view of the simplices, this method may potentially generate data clone. |

## Function Details

### view<a name="view"></a>
!!! function "[[nodiscard]] span&lt;const Vector&lt;IndexT, N + 1&gt;&gt; view() const"

    
    
    Get the const view of the simplices, this method generates no data clone.
    
    :material-keyboard-return: **Return**
    :    span<const Vector<IndexT, N + 1>>
    
    

!!! function "[[nodiscard]] span&lt;Vector&lt;IndexT, N + 1&gt;&gt; view()"

    
    
    Get the non-const view of the simplices, this method may potentially generate data clone.
    
    :material-keyboard-return: **Return**
    :    span<Vector<IndexT, N + 1>>
    
    

