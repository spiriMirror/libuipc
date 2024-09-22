#include <uipc/constitution/constraint.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
Constraint::Constraint() noexcept {}

void Constraint::apply_to(geometry::SimplicialComplex& sc) const
{
    auto constraint_uid = sc.meta().find<U64>(builtin::constraint_uid);
    if(!constraint_uid)
        constraint_uid = sc.meta().create<U64>(builtin::constraint_uid, get_uid());
    else
        geometry::view(*constraint_uid).front() = uid();
}
}  // namespace uipc::constitution
