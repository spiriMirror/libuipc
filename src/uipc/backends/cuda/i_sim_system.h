#pragma once
#include <string_view>

namespace uipc::backend::cuda
{
class ISimSystem
{
  public:
    virtual ~ISimSystem() = default;
    std::string_view name() const noexcept;


  protected:
    virtual void             do_build() = 0;
    virtual std::string_view get_name() const noexcept;

  private:
    friend class SimEngine;
    void build();
};
}  // namespace uipc::backend::cuda
