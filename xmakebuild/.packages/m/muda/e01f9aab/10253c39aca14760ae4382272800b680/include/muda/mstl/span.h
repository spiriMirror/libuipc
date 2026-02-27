#pragma once
#include <muda/tools/platform.h>
#include <muda/mstl/tcb/span.hpp>
#if MUDA_HAS_CXX20
#include <span>
#endif

namespace muda
{
#if MUDA_HAS_CXX20
using std::span;
#else
template <typename T>
using span = tcb::span<T>;
#endif
}  // namespace muda