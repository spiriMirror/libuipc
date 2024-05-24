---
title: SimplicialComplexIO
description: A class for reading and writing simplicial complex. 
generator: doxide
---


# SimplicialComplexIO

**class SimplicialComplexIO**



A class for reading and writing simplicial complex.
 




## Functions

| Name | Description |
| ---- | ----------- |
| [read](#read) | A unified interface for reading a simplicial complex from a file, the file type is determined by the file extension. |
| [read_msh](#read_msh) | Read a tetmesh from a .msh file. |
| [read_obj](#read_obj) | Read a trimesh, linemesh or particles from a .obj file. |

## Function Details

### read<a name="read"></a>
!!! function "[[nodiscard]] SimplicialComplex read(std::string_view file_name)"

    
    
    A unified interface for reading a simplicial complex from a file, the file type is determined by the file extension.
    
    :material-location-enter: **Parameter** `file_name`
    :    The file to read
        :material-keyboard-return: **Return**
    :    SimplicialComplex
        
    

### read_msh<a name="read_msh"></a>
!!! function "[[nodiscard]] SimplicialComplex read_msh(std::string_view file_name)"

    
    
    Read a tetmesh from a .msh file.
    
    :material-location-enter: **Parameter** `file_name`
    :    The file to read
    
    :material-keyboard-return: **Return**
    :    SimplicialComplex
    
    

### read_obj<a name="read_obj"></a>
!!! function "[[nodiscard]] SimplicialComplex read_obj(std::string_view file_name)"

    
    
    Read a trimesh, linemesh or particles from a .obj file.
    
    :material-location-enter: **Parameter** `file_name`
    :    The file to read
    
    :material-keyboard-return: **Return**
    :     SimplicialComplex
    
    

