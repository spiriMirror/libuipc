---
title: ISimplices
description: An abstract class for simplices, special type of topological elements.
generator: doxide
---


# ISimplices

**class ISimplices : public ITopoElements**



An abstract class for simplices, special type of topological elements.




## Functions

| Name | Description |
| ---- | ----------- |
| [dim](#dim) | Get the dimension of the simplices. |

## Function Details

### dim<a name="dim"></a>
!!! function "[[nodiscard]] IndexT dim() const"

    
    
    Get the dimension of the simplices.
    
    E.g. 0 for vertices, 1 for edges, 2 for triangles, 3 for tetrahedra.
    
    :material-keyboard-return: **Return**
    :    IndexT the dimension of the simplices
    
    

