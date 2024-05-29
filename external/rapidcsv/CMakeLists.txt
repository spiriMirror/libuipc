add_library(rapidcsv INTERFACE)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")
target_include_directories(rapidcsv INTERFACE ${RAPIDCSV_INCLUDE_DIRS})
message(STATUS "RAPIDCSV_INCLUDE_DIRS: ${RAPIDCSV_INCLUDE_DIRS}")

