include_directories(.)

file(GLOB metis_sources *.c *.h)
add_library(metis STATIC ${metis_sources})
target_link_libraries(metis GKlib)
target_include_directories(metis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)