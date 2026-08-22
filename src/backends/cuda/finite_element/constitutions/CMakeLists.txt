file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cu" "*.h" "details/*.inl")
target_sources(cuda PRIVATE ${SOURCES})
