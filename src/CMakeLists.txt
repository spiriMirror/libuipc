add_subdirectory(core)

add_subdirectory(geometry)

add_subdirectory(constitution)

add_subdirectory(backends)

add_subdirectory(sanity_check)

add_subdirectory(io)

if(UIPC_BUILD_GUI)
    uipc_warning("This version C++ GUI is not finished yet, waiting for contributions.")
    # gui
    # add_subdirectory(gui)
endif()

add_library(uipc_uipc INTERFACE)
add_library(uipc::uipc ALIAS uipc_uipc)

target_link_libraries(uipc_uipc INTERFACE
    uipc::core
    uipc::geometry
    uipc::constitution
    uipc::io
    uipc::sanity_check
)

if(UIPC_BUILD_PYBIND)
    # py
    add_subdirectory(pybind)
endif()




