---
title: MetaAttributes
description: A wrapper class for the meta attributes of a geometry. 
generator: doxide
---


# MetaAttributes

**class MetaAttributes**



A wrapper class for the meta attributes of a geometry.
     




## Functions

| Name | Description |
| ---- | ----------- |
| [find](#find) | Find an attribute by type and name, if the attribute does not exist, return nullptr.  |
| [create](#create) | Create an attribute with the given name.  |

## Function Details

### create<a name="create"></a>
!!! function "template &lt;typename T&gt; decltype(auto) create(std::string_view name, const T&amp; init_value = {})"

    
    
    Create an attribute with the given name.
             
    
    
    

### find<a name="find"></a>
!!! function "template &lt;typename T&gt; [[nodiscard]] auto find(std::string_view name)"

    
    
    Find an attribute by type and name, if the attribute does not exist, return nullptr.
             
    
    
    

