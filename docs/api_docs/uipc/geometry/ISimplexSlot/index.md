---
title: ISimplexSlot
description: An abstract class representing a simplex slot in an abstract simplicial complex. 
generator: doxide
---


# ISimplexSlot

**class ISimplexSlot**



An abstract class representing a simplex slot in an abstract simplicial complex.
 




## Functions

| Name | Description |
| ---- | ----------- |
| [is_shared](#is_shared) | Check if the underlying simplices is shared. |
| [size](#size) | Get the size of the simplices. |

## Function Details

### is_shared<a name="is_shared"></a>
!!! function "[[nodiscard]] bool is_shared() const"

    
    
    Check if the underlying simplices is shared.
    
    :material-keyboard-return: **Return**
    :    true, if the simplices is shared
    
    :material-keyboard-return: **Return**
    :    false, if the simplices is owned
    
    

### size<a name="size"></a>
!!! function "[[nodiscard]] SizeT size() const"

    
    
    Get the size of the simplices.
    
    :material-keyboard-return: **Return**
    :    the size of the simplices
    
    

