#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/builtin/uid_info.h>
namespace uipc::geometry
{
class UIPC_CORE_API ImplicitGeometry : public Geometry
{
  public:
    ImplicitGeometry();

    const builtin::UIDInfo& uid_info() const noexcept;
    std::string_view        name() const noexcept;

  protected:
    virtual std::string_view get_type() const noexcept override;
    virtual S<IGeometry>     do_clone() const override;
};
}  // namespace uipc::geometry

namespace fmt
{
template <>
struct UIPC_CORE_API formatter<uipc::geometry::ImplicitGeometry>
    : public formatter<std::string_view>
{
    appender format(const uipc::geometry::ImplicitGeometry& geometry,
                    format_context&                         ctx) const;
};
}  // namespace fmt
