# Helper modules.

# Add compiler flags.
if(MSVC)
  set(GKlib_COPTS "/Ox")
  set(GKlib_COPTIONS "-DWIN32 -DMSC -D_CRT_SECURE_NO_DEPRECATE -DUSE_GKREGEX")
elseif(MINGW)
  set(GKlib_COPTS "-DUSE_GKREGEX")
else()
  set(GKlib_COPTIONS "-DLINUX -D_FILE_OFFSET_BITS=64")
endif(MSVC)

if(CYGWIN)
  set(GKlib_COPTIONS "${GKlib_COPTIONS} -DCYGWIN")
endif(CYGWIN)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
# GCC opts.
  set(GKlib_COPTIONS "${GKlib_COPTIONS} -std=c99 -fno-strict-aliasing")

  if(VALGRIND)
    set(GKlib_COPTIONS "${GK_COPTIONS} -march=x86-64 -mtune=generic")
  else()
    set(GKlib_COPTIONS "${GKlib_COPTIONS} -march=native")
  endif(VALGRIND)

  if(NOT MINGW)
      set(GKlib_COPTIONS "${GKlib_COPTIONS} -fPIC")
  endif(NOT MINGW)

# GCC warnings.
  set(GKlib_COPTIONS "${GKlib_COPTIONS} -Werror -Wall -pedantic -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unknown-pragmas -Wno-unused-label")
endif()

if(${CMAKE_C_COMPILER_ID} MATCHES "Sun")
  set(GKlib_COPTIONS "${GKlib_COPTIONS} -xc99")
endif()

# Intel compiler
if(${CMAKE_C_COMPILER_ID} MATCHES "Intel")
  set(GKlib_COPTIONS "${GKlib_COPTIONS} -xHost -std=c99")
endif()

# Set the CPU type 
if(NO_X86)
  set(GKlib_COPTIONS "${GKlib_COPTIONS} -DNO_X86=${NO_X86}")
endif(NO_X86)


# Finally set the official C flags.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GKlib_COPTIONS} ${GKlib_COPTS}")
