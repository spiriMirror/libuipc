---
title: The libuipc API Documentation
description: The libuipc API Documentation
generator: doxide
---


# The libuipc API Documentation

The libuipc API Documentation

:material-package: [uipc](uipc/index.md)
:   

## Macros

| Name | Description |
| ---- | ----------- |
| [REGISTER_SIM_SYSTEM](#REGISTER_SIM_SYSTEM) | Register a SimSystem, which will be automatically created by the SimEngine.  |
| [REGISTER_SIM_SYSTEM_ADVANCED](#REGISTER_SIM_SYSTEM_ADVANCED) | Register a SimSystem with advanced creation logic. |

## Macro Details

### REGISTER_SIM_SYSTEM<a name="REGISTER_SIM_SYSTEM"></a>

!!! macro "#define REGISTER_SIM_SYSTEM(SimSystem)                                             \"

    
    
    Register a SimSystem, which will be automatically created by the SimEngine.
     
    
    
    

### REGISTER_SIM_SYSTEM_ADVANCED<a name="REGISTER_SIM_SYSTEM_ADVANCED"></a>

!!! macro "#define REGISTER_SIM_SYSTEM_ADVANCED                                           \"

    
    
    Register a SimSystem with advanced creation logic.
    
    ```cpp
    REGISTER_SIM_SYSTEM_ADVANCED(
    [](uipc::backend::cuda::SimEngine& engine)
    {
    if(need_to_create)
    return std::make_unique<SimSystem>(engine);
    else
    return nullptr;
    }
    );
    ```
    
    

