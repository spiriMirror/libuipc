---
title: IAttributeSlot
description: An abstract class to represent a geometry attribute slot in a geometry attribute collection.
generator: doxide
---


# IAttributeSlot

**class IAttributeSlot**



An abstract class to represent a geometry attribute slot in a geometry attribute collection.




## Functions

| Name | Description |
| ---- | ----------- |
| [is_shared](#is_shared) | Check if the underlying attribute is shared. |

## Function Details

### is_shared<a name="is_shared"></a>
!!! function "[[nodiscard]] bool  is_shared() const"

    
    
    Check if the underlying attribute is shared.
    
    :material-keyboard-return: **Return**
    :    true, if the underlying attribute is shared, more than one geometry reference to the underlying attribute.
    
    :material-keyboard-return: **Return**
    :    false, if the underlying attribute is owned, only this geometry reference to the underlying attribute.
    
    

