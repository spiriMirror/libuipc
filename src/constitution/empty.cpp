#include <uipc/constitution/empty.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
constexpr U64 EmptyUID = 0ull;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid = EmptyUID, .name = "Empty", .type = string{builtin::FiniteElement}});
    return uids;
}

Empty::Empty(const Json& config) noexcept
    : m_config(config)
{
}

void Empty::apply_to(geometry::SimplicialComplex& sc, Float mass_density, Float thickness) const
{
    Base::apply_to(sc, mass_density, thickness);
}

Json Empty::default_config() noexcept
{
    return Json::object();
}

U64 Empty::get_uid() const noexcept
{
    return EmptyUID;
}
}  // namespace uipc::constitution
