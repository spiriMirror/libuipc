#pragma once
#include <cuda.h>
namespace muda
{
struct is_cuda_arch
{
#ifdef __CUDA_ARCH__
	constexpr static bool value = true;
#else
	constexpr static bool value = false;
#endif
};

constexpr bool is_cuda_arch_v = is_cuda_arch::value;
}  // namespace muda