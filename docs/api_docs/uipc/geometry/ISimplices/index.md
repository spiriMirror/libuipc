---
title: ISimplices
description: An abstract class for simplices, special type of topological elements.
generator: doxide
---


# ISimplices

**class  ISimplices : public ITopoElements**



An abstract class for simplices, special type of topological elements.




## Functions

| Name | Description |
| ---- | ----------- |
| [dim](#dim) | Get the dimension of the simplices. |

## Function Details

### dim<a name="dim"></a>
!!! function "[[nodiscard]] IndexT dim() const"

    
    
    Get the dimension of the simplices.
    
    | Dimension | Type of simplices |
    |-----------|-------------------|
    |    0      | Vertices          |
    |    1      | Edges             |
    |    2      | Triangles         |
    |    3      | Tetrahedra        |
    
    :material-keyboard-return: **Return**
    :    the dimension of the simplices
    
    

