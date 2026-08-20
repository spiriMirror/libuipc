file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "*.cpp" "*.h" "*.inl")
target_sources(uipc_core PRIVATE ${SOURCES})
