#include <uipc/world/constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::world
{
U64 IConstitution::uid() const
{
    return get_uid();
}

void IConstitution::apply_to(geometry::Geometry& geo) const
{
    auto meta = geo.meta();
    auto P    = meta.find<U64>(builtin::constitution);

    if(!P)
        P = meta.create<U64>(builtin::constitution_uid, uid());
    else
        geometry::view(*P).front() = uid();
}
}  // namespace uipc::world