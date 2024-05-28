---
title: Attribute
description: Template class to represent a geometry attribute of type T.
generator: doxide
---


# Attribute

**template &lt;typename T&gt; class Attribute : public IAttribute**



Template class to represent a geometry attribute of type T.

:material-code-tags: **Template parameter** `T`
:    The type of the attribute values.
    


## Functions

| Name | Description |
| ---- | ----------- |
| [view](#view) | Get a non-const view of the attribute values. This method may potentially clone the attribute data. |
| [view](#view) | Get a const view of the attribute values. This method gerantees no data cloning. |

## Function Details

### view<a name="view"></a>
!!! function "[[nodiscard]] span&lt;T&gt; view(Attribute&lt;T&gt;&amp; a) noexcept"

    
    
    Get a non-const view of the attribute values. This method may potentially clone the attribute data.
    
    !!! note
        
        Always consider using the const member method if the attribute data is not going to be modified.
    
    :material-keyboard-return: **Return**
    :    `span<T>`
    
    

!!! function "[[nodiscard]] span&lt;const T&gt; view() const noexcept"

    
    
    Get a const view of the attribute values. This method gerantees no data cloning.
    
    :material-keyboard-return: **Return**
    :    `span<const T>`
    
    

