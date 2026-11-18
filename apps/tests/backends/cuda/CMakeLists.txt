# NVIDIA contributions: Windows builds without the tests under internals/,
# which link against symbols the backend DLL does not export.
#
# The NVIDIA notice applies to the contributions described above, not to the
# pre-existing libuipc or other contributors' code.

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

enable_language(CUDA)

# CUDA >= 13 CCCL requires the standard-conforming MSVC preprocessor for host code.
if(MSVC AND CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 13.0)
    uipc_info("CUDA >= 13 with MSVC: adding /Zc:preprocessor to CUDA host compilation.")
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>
                        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
endif()

# extended device lambdas + relaxed constexpr are required by the cuda_tool
# launch/dense math used throughout the backend (and fmt's constexpr literals).
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
                    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)

file(GLOB_RECURSE SOURCE CONFIGURE_DEPENDS "*.cpp" "*.h" "*.cu" ".inl")
# The tests under internals/ call host code compiled into the CUDA backend
# (kernel launchers, linear-system internals) and link against symbols of the
# backend library itself. A Linux shared library exports them; the backend DLL
# on Windows exports only its module entry points, so Windows builds leave
# these tests out.
if(WIN32)
    list(FILTER SOURCE EXCLUDE REGEX "/backends/cuda/internals/")
endif()
uipc_add_test(backend_cuda ${SOURCE})
target_link_libraries(backend_cuda PRIVATE uipc::backend::cuda)
target_compile_options(backend_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>: --diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,55,221,1028>)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(UIPC_GITHUB_ACTIONS)
        set(CMAKE_CUDA_ARCHITECTURES 75)
    else()
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()
endif()

set_target_properties(backend_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_INCLUDE_WHAT_YOU_USE ON
)
uipc_set_target_cuda_architectures(backend_cuda "${CMAKE_CUDA_ARCHITECTURES}")
