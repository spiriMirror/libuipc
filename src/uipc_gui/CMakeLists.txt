find_package(bgfx CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(GLFW3 CONFIG REQUIRED)

add_library(uipc_gui STATIC)

add_subdirectory(common)



file(GLOB SOURCE "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_gui PRIVATE ${SOURCE})
target_link_libraries(uipc_gui PUBLIC
    imgui::imgui
    bgfx::bx 
    bgfx::bgfx 
    bgfx::bimg 
    bgfx::bimg_decode
    glfw
)
target_link_libraries(uipc_gui PUBLIC uipc)
file(GLOB_RECURSE ALL_SOURCE "*.cpp" "*.h" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${ALL_SOURCE})

