cmake_minimum_required(VERSION 3.10)

# ...
project(GKlib
  VERSION   0.0.1
  LANGUAGES C)

# include required CMake modules
include(CheckCCompilerFlag)
include(CheckCSourceCompiles)
include(CheckFunctionExists)
include(CheckIncludeFile)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

#-------------------------------------------------------------------------------
# OPTIONS
#-------------------------------------------------------------------------------
option(ASSERT "turn asserts on" OFF)
option(ASSERT2 "additional assertions" OFF)
option(DEBUG "add debugging support" OFF)
option(GPROF "add gprof support" OFF)
option(GDB "add gdb support" OFF)
option(GKRAND "enable GKRAND support" OFF)
option(GKREGEX "enable GKREGEX support" OFF)
option(OPENMP "enable OpenMP support" OFF)
option(PCRE "enable PCRE support" OFF)
option(VALGRID "enable valgrind support" OFF)
option(NO_X86 "enable no-x86 support" OFF)
option(SHARED "enable shared support" OFF)


#-------------------------------------------------------------------------------
# LIBRARY configuration
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}
  PRIVATE src/b64.c src/blas.c src/cache.c src/csr.c src/error.c src/evaluate.c
          src/fkvkselect.c src/fs.c src/getopt.c src/gk_util.c src/gkregex.c
          src/graph.c src/htable.c src/io.c src/itemsets.c src/mcore.c
          src/memory.c src/pqueue.c src/random.c src/rw.c src/seq.c src/sort.c
          src/string.c src/timers.c src/tokenizer.c
          # these are only included below so that they appear when using IDEs
          include/GKlib.h include/gk_arch.h include/gk_defs.h
          include/gk_externs.h include/gk_getopt.h include/gk_macros.h 
          include/gk_mkblas.h include/gk_mkmemory.h include/gk_mkpqueue.h
          include/gk_mkpqueue2.h include/gk_mkrandom.h include/gk_mksort.h 
          include/gk_mkutils.h include/gk_proto.h include/gk_struct.h
          include/gk_types.h include/gkregex.h include/gk_ms_inttypes.h 
          include/gk_ms_stat.h include/gk_ms_stdint.h
          # the following are shims for win32 systems
          $<$<PLATFORM_ID:Windows>:src/win32/adapt.c
                                   include/win32/adapt.h>)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC $<$<PLATFORM_ID:Linux>:LINUX>)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<$<PLATFORM_ID:Windows>:${CMAKE_CURRENT_SOURCE_DIR}/include/win32>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
  PUBLIC $<$<NOT:$<C_COMPILER_ID:MSVC>>:m>)

set_target_properties(${PROJECT_NAME} PROPERTIES
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION   ${PROJECT_VERSION})

add_library(GKlib::GKlib ALIAS ${PROJECT_NAME})

#-------------------------------------------------------------------------------
# OPTIONS configuration
#-------------------------------------------------------------------------------
target_compile_definitions(${PROJECT_NAME}
  PUBLIC $<$<NOT:$<BOOL:${ASSERT}>>:NDEBUG>
         $<$<NOT:$<BOOL:${ASSERT2}>>:NDEBUG2>
         $<$<BOOL:${DEBUG}>:DEBUG>
         $<$<BOOL:${GKRAND}>:GKRAND>
         $<$<BOOL:${NO_X86}>:NO_X86>
         )

#-------------------------------------------------------------------------------
# FEATURE AVAILABILITY checks
#-------------------------------------------------------------------------------
check_include_file(execinfo.h HAVE_EXECINFO_H)
check_function_exists(getline HAVE_GETLINE)

# regular expressions
if(PCRE)
  check_include_file(pcreposix.h HAVE_PCREPOSIX_H)
  if(NOT HAVE_PCREPOSIX_H)
    message(WARNING "PCRE was requested, but is not available")
  endif()
endif()
if(NOT HAVE_PCREPOSIX_H)
  check_include_file(regex.h HAVE_REGEX_H)
  if(NOT HAVE_REGEX_H)
    set(USE_GKREGEX ON)
  endif()
endif()

# profiling support
if(GPROF)
  check_c_compiler_flag("-pg" HAVE_GPROF_SUPPORT)
  if(NOT HAVE_GPROF_SUPPORT)
    message(WARNING "GPROF support was requested, but is not available")
  endif()
endif()

# profiling support
if(GDB|DEBUG)
  check_c_compiler_flag("-g" HAVE_GDB_SUPPORT)
  if(NOT HAVE_GDB_SUPPORT)
    message(WARNING "GDB support was requested, but is not available")
  endif()
endif()

# openmp support
if(OPENMP)
  find_package(OpenMP)
  if(NOT OpenMP_C_FOUND)
    message(WARNING "OpenMP was requested, but is not available")
  endif()
endif()

# thread local storage
if(NOT DEFINED HAVE_TLS)
  set(TLS_NAME "" CACHE INTERNAL "Thread local keyword")
  foreach(tls_name "__thread" "__declspec(thread)")
    unset(HAVE_TLS CACHE)
    check_c_source_compiles("${tls_name} int x; int main(void) { return 0; }"
      HAVE_TLS)
    if (HAVE_TLS)
      set(TLS_NAME ${tls_name} CACHE INTERNAL "Thread local keyword")
      break()
    else()
    endif()
  endforeach()
endif()

target_compile_definitions(${PROJECT_NAME}
  PUBLIC $<$<BOOL:${HAVE_EXECINFO_H}>:HAVE_EXEC_INFO_H>
         $<$<BOOL:${PCRE}>:USE_PCRE>
         $<$<AND:$<BOOL:${PCRE}>,$<BOOL:${HAVE_PCREPOSIX_H}>>:HAVE_PCREPOSIX_H>
         $<$<BOOL:${HAVE_REGEX_H}>:HAVE_REGEX_H>
         $<$<BOOL:${USE_GKREGEX}>:USE_GKREGEX>
         $<$<BOOL:${HAVE_GETLINE}>:HAVE_GETLINE>
         __thread=${TLS_NAME})

target_compile_options(${PROJECT_NAME}
  PUBLIC $<$<AND:$<BOOL:${GPROF}>,$<BOOL:${HAVE_GPROF_SUPPORT}>>:-pg>)

target_compile_options(${PROJECT_NAME}
  PUBLIC $<$<AND:$<OR:$<BOOL:${DEBUG}>,$<BOOL:${GDB}>>,$<BOOL:${HAVE_GDB_SUPPORT}>>:-g>)

target_compile_options(${PROJECT_NAME}
  PUBLIC $<$<NOT:$<OR:$<BOOL:${DEBUG}>,$<BOOL:${GDB}>>>:-O3>)

target_link_libraries(${PROJECT_NAME}
  PUBLIC $<$<BOOL:${OpenMP_C_FOUND}>:OpenMP::OpenMP_C>)

