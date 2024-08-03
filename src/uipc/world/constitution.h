#pragma once
#include <string_view>
#include <uipc/geometry/geometry.h>
#include <uipc/builtin/constitution_uid_register.h>
namespace uipc::world
{
enum class ConstitutionType
{
    AffineBody,
    FiniteElement,
};

class UIPC_CORE_API IConstitution
{
  public:
    virtual ~IConstitution() = default;
    U64               uid() const noexcept;
    std::string_view  name() const noexcept;
    ConstitutionType type() const noexcept;


  protected:
    void                      apply_to(geometry::Geometry& geo) const;
    virtual U64               get_uid() const noexcept  = 0;
    virtual std::string_view  get_name() const noexcept = 0;
    virtual ConstitutionType get_type() const noexcept = 0;
};
}  // namespace uipc::world
