#pragma once
#include <string_view>
#include <uipc/common/json.h>
#include <uipc/common/exception.h>
namespace uipc::backend
{
class ISimSystem
{
  public:
    virtual ~ISimSystem() = default;
    std::string_view name() const noexcept;
    bool             is_engine_aware() const noexcept;
    bool             is_valid() const noexcept;
    bool             is_building() const noexcept;
    Json             to_json() const;

  protected:
    virtual void             do_build() = 0;
    virtual std::string_view get_name() const noexcept;

  private:
    friend class SimEngine;
    friend class SimSystemCollection;
    friend class SimSystem;

    void build();
    void make_engine_aware();
    void invalidate() noexcept;

    virtual void set_engine_aware() noexcept       = 0;
    virtual bool get_engine_aware() const noexcept = 0;
    virtual void set_invalid() noexcept            = 0;
    virtual bool get_valid() const noexcept        = 0;
    virtual Json do_to_json() const                = 0;
    virtual void set_building(bool b) noexcept     = 0;
    virtual bool get_is_building() const noexcept  = 0;
};

class SimSystemException : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::backend
