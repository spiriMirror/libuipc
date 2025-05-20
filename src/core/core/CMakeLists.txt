file(GLOB_RECURSE SOURCES "*.cpp" "*.h" "*.inl")
target_sources(uipc_core PRIVATE ${SOURCES})
