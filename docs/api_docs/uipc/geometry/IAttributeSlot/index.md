---
title: IAttributeSlot
description: An abstract class to represent a geometries attribute slot in a geometries attribute collection.
generator: doxide
---


# IAttributeSlot

**class  IAttributeSlot**



An abstract class to represent a geometries attribute slot in a geometries attribute collection.




## Functions

| Name | Description |
| ---- | ----------- |
| [name](#name) | Get the name of the attribute slot.  |
| [type_name](#type_name) | Get the type name of data stored in the attribute slot.  |
| [allow_destroy](#allow_destroy) | Check if the underlying attribute is allowed to be destroyed.  |
| [is_shared](#is_shared) | Check if the underlying attribute is shared. |

## Function Details

### allow_destroy<a name="allow_destroy"></a>
!!! function "[[nodiscard]] bool allow_destroy() const noexcept"

    
    
    Check if the underlying attribute is allowed to be destroyed.
         
    
    
    

### is_shared<a name="is_shared"></a>
!!! function "[[nodiscard]] bool  is_shared() const noexcept"

    
    
    Check if the underlying attribute is shared.
    
    :material-keyboard-return: **Return**
    :    true, if the underlying attribute is shared, more than one geometries reference to the underlying attribute.
    
    :material-keyboard-return: **Return**
    :    false, if the underlying attribute is owned, only this geometries reference to the underlying attribute.
    
    

### name<a name="name"></a>
!!! function "[[nodiscard]] std::string_view name() const noexcept"

    
    
    Get the name of the attribute slot.
         
    
    
    

### type_name<a name="type_name"></a>
!!! function "[[nodiscard]] std::string_view type_name() const noexcept"

    
    
    Get the type name of data stored in the attribute slot.
         
    
    
    

