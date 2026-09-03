file(GLOB METIS_CPP_SOURCES CONFIGURE_DEPENDS "*.cpp")

add_library(uipc_metis STATIC ${METIS_CPP_SOURCES})

target_compile_features(uipc_metis PRIVATE cxx_std_20)
target_include_directories(
    uipc_metis
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_compile_definitions(uipc_metis PRIVATE NDEBUG)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(uipc_metis PRIVATE LINUX)
    target_link_libraries(uipc_metis PRIVATE m)
endif()

set_target_properties(
    uipc_metis
    PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        FOLDER "components"
)

file(GLOB_RECURSE METIS_SOURCE_GROUP_FILES CONFIGURE_DEPENDS "*.h" "*.cpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${METIS_SOURCE_GROUP_FILES})
