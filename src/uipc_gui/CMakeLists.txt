find_package(bgfx CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(GLFW3 CONFIG REQUIRED)

add_library(uipc_gui STATIC)
add_library(uipc::gui ALIAS uipc_gui)

# ------------------------------------------------------------------------------
# target sources
# ------------------------------------------------------------------------------
file(GLOB SOURCE "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_gui PRIVATE ${SOURCE})

# ------------------------------------------------------------------------------
# link libraries
# ------------------------------------------------------------------------------
target_link_libraries(uipc_gui PRIVATE
    imgui::imgui
    bgfx::bx 
    bgfx::bgfx 
    bgfx::bimg 
    bgfx::bimg_decode
    glfw
)
# make the uipc::uipc PUBLIC so that the client can directly use it
target_link_libraries(uipc_gui PUBLIC uipc::uipc)
include(uipc_utils)
# add the backend dependency, so that the backend will be built before the gui
uipc_add_backend_dependency(uipc_gui)


# ------------------------------------------------------------------------------
# add subdirectories
# ------------------------------------------------------------------------------
add_subdirectory(common)


# ------------------------------------------------------------------------------
# Group files for IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE ALL_SOURCE "*.cpp" "*.h" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${ALL_SOURCE})

