find_package(PxrBoostPython REQUIRED CONFIG)

file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(pyuipc PRIVATE ${SOURCES})

target_link_libraries(pyuipc PRIVATE PxrBoostPython::PxrBoostPython)