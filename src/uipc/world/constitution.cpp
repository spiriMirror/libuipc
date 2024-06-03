#include <uipc/world/constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::world
{
U64 IConstitution::uid() const noexcept
{
    return get_uid();
}

std::string_view IConstitution::name() const noexcept
{
    return get_name();
}

void IConstitution::apply_to(geometry::Geometry& geo) const
{
    auto P = geo.meta().find<U64>(builtin::constitution);

    if(!P)
        P = geo.meta().create<U64>(builtin::constitution_uid, uid());
    else
        geometry::view(*P).front() = uid();
}
}  // namespace uipc::world
