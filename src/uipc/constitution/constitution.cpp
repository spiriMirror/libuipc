#include <uipc/constitution/constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
U64 IConstitution::uid() const noexcept
{
    return get_uid();
}

std::string_view IConstitution::name() const noexcept
{
    return uid_info().name;
}

ConstitutionType IConstitution::type() const noexcept
{
    return get_type();
}

const builtin::UIDInfo& IConstitution::uid_info() const noexcept
{
    return builtin::ConstitutionUIDCollection::instance().find(get_uid());
}

void IConstitution::apply_to(geometry::Geometry& geo) const
{
    auto P = geo.meta().find<U64>(builtin::constitution_uid);

    if(!P)
        P = geo.meta().create<U64>(builtin::constitution_uid, uid());
    else
        geometry::view(*P).front() = uid();
}
}  // namespace uipc::constitution
