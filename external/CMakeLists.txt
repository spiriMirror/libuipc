if (UIPC_WITH_CUDA_BACKEND)
    uipc_init_submodule(muda)
    # force MUDA_BUILD_EXAMPLE to be OFF
    set(MUDA_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    if (NOT DEFINED CMAKE_ABSOLUTE_DESTINATION_FILES)
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()
    add_subdirectory(muda)
    set_target_properties(muda PROPERTIES FOLDER "external")
endif()

uipc_init_submodule(tetgen)
add_subdirectory(tetgen)
set_target_properties(tetgen PROPERTIES FOLDER "external")

add_subdirectory(octree)


