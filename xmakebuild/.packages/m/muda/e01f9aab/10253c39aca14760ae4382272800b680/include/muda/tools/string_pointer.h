#pragma once
#include <muda/muda_def.h>

namespace muda::details
{
class StringPointer
{
  public:
    char*        device_string = nullptr;
    char*        host_string   = nullptr;
    unsigned int length        = 0;

    MUDA_INLINE MUDA_GENERIC const char* auto_select() const MUDA_NOEXCEPT
    {
#ifdef __CUDA_ARCH__
        return device_string;
#else
        return host_string;
#endif  // __CUDA_ARCH__
    }
};
}  // namespace muda