# 2025.10.9
# We remove all submodules from git and manage them via vcpkg instead.
# For fast-start-development, we provide a helper function to initialize submodules.
# Once the dependency is stabilized, we will make it a vcpkg dependency and remove the submodule initialization.

# Submodule Usage 
# uipc_init_submodule(my_lib) # auto init submodule if not exist
# add_subdirectory(my_lib)    # include the submodule cmake project
# set_target_properties(my_lib PROPERTIES FOLDER "external") # optional, organize in IDEs

if (UIPC_WITH_CUDA_BACKEND)
    uipc_init_submodule(muda)
    # Turn off example builds in muda
    set(MUDA_BUILD_EXAMPLE OFF CACHE BOOL "Build muda example" FORCE)
    add_subdirectory(muda)
    add_library(muda::muda ALIAS muda)
    set_target_properties(muda PROPERTIES FOLDER "external")
endif()
