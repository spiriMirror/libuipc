#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/smart_pointer.h>

namespace uipc::core
{
class UIPC_CORE_API IFeature
{
  public:
    virtual ~IFeature() = default;
    std::string_view name() const;
    std::string_view type_name() const;

  protected:
    virtual std::string_view get_name() const      = 0;
    virtual std::string_view get_type_name() const = 0;
};

class UIPC_CORE_API Feature : public IFeature
{
  public:
  private:
    virtual std::string_view get_type_name() const override;
    mutable std::string      m_type_name;
};
}  // namespace uipc::core
