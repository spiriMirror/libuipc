#include <uipc/constitution/constraint.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/set.h>
namespace uipc::constitution
{
Constraint::Constraint() noexcept {}

void Constraint::apply_to(geometry::SimplicialComplex& sc) const
{
    auto uids = sc.meta().find<VectorXu64>(builtin::constraint_uids);
    if(!uids)
    {
        uids = sc.meta().create<VectorXu64>(builtin::constraint_uids);
    }

    // add uid
    auto&    vs = geometry::view(*uids).front();
    set<U64> uids_set(vs.begin(), vs.end());
    uids_set.insert(uid());
    vs.resize(uids_set.size());
    std::ranges::copy(uids_set, vs.begin());
}
}  // namespace uipc::constitution
