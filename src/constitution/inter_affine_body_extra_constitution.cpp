#include <uipc/constitution/inter_affine_body_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/set.h>

namespace uipc::constitution
{
void InterAffineBodyExtraConstitution::apply_to(geometry::SimplicialComplex& sc) const
{
    auto uids = sc.meta().find<VectorXu64>(builtin::extra_constitution_uids);
    if(!uids)
        uids = sc.meta().create<VectorXu64>(builtin::extra_constitution_uids);

    auto&    vs = geometry::view(*uids).front();
    set<U64> uid_set(vs.begin(), vs.end());
    uid_set.insert(uid());
    vs.resize(uid_set.size());
    std::ranges::copy(uid_set, vs.begin());
}
}  // namespace uipc::constitution
