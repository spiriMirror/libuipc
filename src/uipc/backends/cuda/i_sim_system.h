#pragma once
#include <string_view>
#include <uipc/common/json.h>
namespace uipc::backend::cuda
{
class ISimSystem
{
  public:
    virtual ~ISimSystem() = default;
    std::string_view name() const noexcept;
    bool             is_engine_aware() const noexcept;
    Json             to_json() const;

  protected:
    virtual void             do_build() = 0;
    virtual std::string_view get_name() const noexcept;
    virtual void             set_engine_aware() noexcept       = 0;
    virtual bool             get_engine_aware() const noexcept = 0;
    virtual Json             do_to_json() const                = 0;

  private:
    friend class SimEngine;
    void build();
    void make_engine_aware();
};
}  // namespace uipc::backend::cuda
