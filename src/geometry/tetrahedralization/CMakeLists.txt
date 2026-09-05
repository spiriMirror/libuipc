file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp" "*.h")
target_sources(uipc_geometry PRIVATE ${SOURCES})

# Error-free expansion arithmetic requires IEEE rounding without reassociation.
if(MSVC)
    set_source_files_properties(${SOURCES} TARGET_DIRECTORY uipc_geometry
        PROPERTIES COMPILE_OPTIONS "/fp:strict")
else()
    set_source_files_properties(${SOURCES} TARGET_DIRECTORY uipc_geometry PROPERTIES
        COMPILE_OPTIONS "-fno-fast-math;-ffp-contract=off")
endif()
