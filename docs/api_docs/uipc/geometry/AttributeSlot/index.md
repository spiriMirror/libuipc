---
title: AttributeSlot
description: Template class to represent a geometry attribute slot of type T in a geometry attribute collection.
generator: doxide
---


# AttributeSlot

**template &lt;typename T&gt; class AttributeSlot final : public IAttributeSlot**



Template class to represent a geometry attribute slot of type T in a geometry attribute collection.

:material-code-tags: **Template parameter** `T`
:    The type of the attribute values.
    


## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | A shortcut to get the non-const attribute values. |
| [view](#view) | A shortcut to get the const attribute values. |

## Function Details

### view<a name="view"></a>
!!! function "[[nodiscard]] span&lt;T&gt; view()"

    
    
    A shortcut to get the non-const attribute values.
    
    :material-keyboard-return: **Return**
    :    `span<T>`
    
    :material-eye-outline: **See**
    :    [Attribute](../Attribute/index.md#Attribute)
    
    

!!! function "[[nodiscard]] span&lt;const T&gt; view() const"

    
    
    A shortcut to get the const attribute values.
    
    :material-keyboard-return: **Return**
    :    `span<const T>`
    
    

