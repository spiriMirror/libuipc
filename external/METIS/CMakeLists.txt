cmake_minimum_required(VERSION 3.18)

#add_library(METIS INTERFACE)
#add_library(METIS::mts ALIAS METIS)


include_directories(include)
add_subdirectory(libmetis)

#add_dependencies(METIS metis)

#set_target_properties(metis PROPERTIES FOLDER "METIS")
#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC_LIST})
