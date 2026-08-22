file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp" "*.h" "details/*.inl")
target_sources(pyuipc PRIVATE ${SOURCES})