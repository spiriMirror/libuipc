---
title: AttributeCollection
description: A collection of geometry attributes.
generator: doxide
---


# AttributeCollection

**class AttributeCollection**



A collection of geometry attributes.

All geometry attributes in the collection always have the same size.



## Functions

| Name | Description |
| ---- | ----------- |
| [create](#create) | Create a new attribute slot of type T with a given name. |
| [share](#share) | Share the underlying attribute of the given slot with a new name. |
| [share](#share) | Template version of share.  |
| [destroy](#destroy) | Remove the attribute slot with the given name. |
| [find](#find) | Find the attribute slot with the given name. |
| [find](#find) | const version of find.  |
| [find](#find) | Template version of find.  |
| [find](#find) | Template const version of find.  |
| [resize](#resize) | Resize all attribute slots to the given size. |
| [size](#size) | Get the size of the attribute slots.  |
| [clear](#clear) | clear the underlying attribute values of all attribute slots, the attribute slots will not be destroyed. |
| [reserve](#reserve) | Reserve memory for all attribute slots. |

## Function Details

### clear<a name="clear"></a>
!!! function "void clear()"

    
    
    clear the underlying attribute values of all attribute slots, the attribute slots will not be destroyed.
    
    !!! note
         This method may generate data clones.
        
    

### create<a name="create"></a>
!!! function "template &lt;typename T, bool AllowDestroy = true&gt; P&lt;AttributeSlot&lt;T&gt;&gt; create(std::string_view name, const T&amp; default_value = {})"

    
    
    Create a new attribute slot of type T with a given name.
    
    :material-code-tags: **Template parameter** `T`
    :    The type of the attribute values.
    
    :material-location-enter: **Parameter** `name`
    :    The name of the attribute slot.
    
    :material-keyboard-return: **Return**
    :    The created attribute slot.
    
    

### destroy<a name="destroy"></a>
!!! function "void destroy(std::string_view name)"

    
    
    Remove the attribute slot with the given name.
    
    The underlying attribute will not be destroyed if it is shared by other attribute slots.
    
    !!! danger
         Accessing the removed attribute slot will cause undefined behavior.
        It's user's responsibility to ensure that the removed attribute slot is not accessed.
    
    :material-location-enter: **Parameter** `name`
    :   
        
    

### find<a name="find"></a>
!!! function "[[nodiscard]] P&lt;IAttributeSlot&gt; find(std::string_view name)"

    
    
    Find the attribute slot with the given name.
    
    :material-location-enter: **Parameter** `name`
    :    The name of the attribute slot.
    
    :material-keyboard-return: **Return**
    :    The attribute slot with the given name.
    
    :material-keyboard-return: **Return**
    :    nullptr if the attribute slot with the given name does not exist.
    
    

!!! function "[[nodiscard]] P&lt;const IAttributeSlot&gt; find(std::string_view name) const"

    
    
    const version of find.
         
    
    
    

!!! function "template &lt;typename T&gt; [[nodiscard]] P&lt;AttributeSlot&lt;T&gt;&gt; find(std::string_view name)"

    
    
    Template version of find.
         
    
    
    

!!! function "template &lt;typename T&gt; [[nodiscard]] P&lt;const AttributeSlot&lt;T&gt;&gt; find(std::string_view name) const"

    
    
    Template const version of find.
         
    
    
    

### reserve<a name="reserve"></a>
!!! function "void reserve(size_t N)"

    
    
    Reserve memory for all attribute slots.
    
    !!! note
         This method generates no data clone. But the memory of the underlying attribute values may be reallocated.
        
    

### resize<a name="resize"></a>
!!! function "void resize(size_t N)"

    
    
    Resize all attribute slots to the given size.
    
    !!! note
         This method may generate data clones.
        
    

### share<a name="share"></a>
!!! function "IAttributeSlot&amp; share(std::string_view name, const IAttributeSlot&amp; slot)"

    
    
    Share the underlying attribute of the given slot with a new name.
    
    The slot may be from another geometry attribute collection or just current geometry attribute collection.
    
    :material-location-enter: **Parameter** `name`
    :    The name of the attribute slot.
    
    :material-location-enter: **Parameter** `slot`
    :    The slot brings the underlying attribute.
    
    :material-keyboard-return: **Return**
    :    The new created attribute slot.
    
    :material-alert-circle-outline: **Throw**
    :    AttributeAlreadyExist if the attribute with the given name already exists.
    
    

!!! function "template &lt;typename T&gt; AttributeSlot&lt;T&gt;&amp; share(std::string_view name, const AttributeSlot&lt;T&gt;&amp; slot)"

    
    
    Template version of share.
         
    
    
    

### size<a name="size"></a>
!!! function "[[nodiscard]] size_t size() const"

    
    
    Get the size of the attribute slots.
         
    
    
    

