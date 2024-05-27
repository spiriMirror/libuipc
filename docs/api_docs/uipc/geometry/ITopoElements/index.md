---
title: ITopoElements
description: An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements. 
generator: doxide
---


# ITopoElements

**class ITopoElements**



An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements.
 




## Functions

| Name | Description |
| ---- | ----------- |
| [tuple_size](#tuple_size) | Get the number of indices in a tuple. |
| [tuple_size](#tuple_size) | Get the number of indices in the i-th tuple. |
| [size](#size) | Get the number of tuples. |
| [resize](#resize) | Resize the number of tuples. |
| [clear](#clear) | Clear the topology.  |
| [reserve](#reserve) | Reserve the memory for the topology. |
| [clone](#clone) | Clone the topology. |

## Function Details

### clear<a name="clear"></a>
!!! function "void                           clear()"

    
    
    Clear the topology.
         
    
    
    

### clone<a name="clone"></a>
!!! function "[[nodiscard]] S&lt;ITopoElements&gt; clone() const"

    
    
    Clone the topology.
    
    :material-keyboard-return: **Return**
    :    A shared pointer to the cloned topology
    
    

### reserve<a name="reserve"></a>
!!! function "void                           reserve(SizeT N)"

    
    
    Reserve the memory for the topology.
    
    :material-location-enter: **Parameter** `SizeT`
    :    N
        
    

### resize<a name="resize"></a>
!!! function "void                           resize(SizeT N)"

    
    
    Resize the number of tuples.
    
    :material-location-enter: **Parameter** `SizeT`
    :    N
        
    

### size<a name="size"></a>
!!! function "[[nodiscard]] SizeT            size() const"

    
    
    Get the number of tuples.
    
    :material-keyboard-return: **Return**
    :    The number of tuples
    
    

### tuple_size<a name="tuple_size"></a>
!!! function "[[nodiscard]] SizeT            tuple_size() const"

    
    
    Get the number of indices in a tuple.
    
    If return ~0ull, it means the number of indices in a tuple is not fixed. user should call tuple_size(IndexT i) to get the number of indices in a tuple.
    
    :material-keyboard-return: **Return**
    :    The number of indices in a tuple
    
    

!!! function "[[nodiscard]] SizeT            tuple_size(IndexT i) const"

    
    
    Get the number of indices in the i-th tuple.
    
    :material-location-enter: **Parameter** `IndexT`
    :    i
    
    :material-keyboard-return: **Return**
    :    The number of indices in the i-th tuple
    
    

