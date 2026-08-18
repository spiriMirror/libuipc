#pragma once
// Compatibility header replacing the muda umbrella include.
// The device utilities formerly provided by the external muda submodule are now
// vendored under cuda_tool/muda/. This header forwards to the vendored muda.h plus
// the ext/ modules the backend uses (eigen small-matrix math + linear_system
// sparse formats), so existing backend code gets the exact same implementation as
// before (behavior unchanged).
#include <muda/muda.h>
#include <muda/ext/eigen.h>
#include <muda/ext/eigen/evd.h>
#include <muda/ext/eigen/svd.h>
#include <muda/ext/linear_system.h>
// cub device algorithms pull in device-only PTX intrinsics; only include them when
// compiling CUDA device code so host (.cpp) translation units don't choke on them.
#ifdef __CUDACC__
#include <muda/cub/cub_device.h>
#endif
