#pragma once
#include <string_view>

namespace uipc::backend::cuda
{
class ISimSystem
{
  public:
    virtual ~ISimSystem() = default;
    std::string_view         name() const noexcept;
    virtual std::string_view get_name() const noexcept;
    virtual void             build() = 0;
};
}  // namespace uipc::backend::cuda
