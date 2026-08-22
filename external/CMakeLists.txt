# 2025.10.9
# We remove all submodules from git and manage them via vcpkg instead.
# For fast-start-development, we provide a helper function to initialize submodules.
# Once the dependency is stabilized, we will make it a vcpkg dependency and remove the submodule initialization.

# Submodule Usage 
# uipc_init_submodule(my_lib) # auto init submodule if not exist
# add_subdirectory(my_lib)    # include the submodule cmake project
# set_target_properties(my_lib PROPERTIES FOLDER "external") # optional, organize in IDEs

# GKlib + METIS (bundled from Stiff-GIPC for graph partitioning)
add_subdirectory(GKlib)
add_subdirectory(METIS)
set_target_properties(GKlib PROPERTIES FOLDER "external")
set_target_properties(metis PROPERTIES FOLDER "external")

# muda is no longer a submodule: its (header-only) source is vendored under
# src/backends/cuda/cuda_tool/muda/ and used directly by the cuda backend.
