#pragma once
#include <utility>
namespace muda::details::buffer
{
class BufferInfoAccessor
{
  public:
    template <typename BufferView>
    static auto cuda_pitched_ptr(BufferView&& b)
    {
        return std::forward<BufferView>(b).cuda_pitched_ptr();
    }
};
}  // namespace muda::details::buffer
