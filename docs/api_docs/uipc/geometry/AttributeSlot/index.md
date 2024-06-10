---
title: AttributeSlot
description: Template class to represent a geometries attribute slot of type T in a geometries attribute collection.
generator: doxide
---


# AttributeSlot

**template &lt;typename T&gt; class AttributeSlot final : public IAttributeSlot**



Template class to represent a geometries attribute slot of type T in a geometries attribute collection.

:material-code-tags: **Template parameter** `T`
:    The type of the attribute values.
    


## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get the non-const attribute values. |
| [view](#view) | Get the const attribute values. |

## Function Details

### view<a name="view"></a>
!!! function "span&lt;U&gt; view(AttributeSlot&lt;U&gt;&amp; slot)"

    
    
    Get the non-const attribute values.
    
    :material-keyboard-return: **Return**
    :    `span<T>`
    
    :material-eye-outline: **See**
    :    [Attribute](../Attribute/index.md#Attribute)
    
    

!!! function "[[nodiscard]] span&lt;const T&gt; view() const noexcept"

    
    
    Get the const attribute values.
    
    :material-keyboard-return: **Return**
    :    `span<const T>`
    
    

