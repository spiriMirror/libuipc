#pragma once
#include <uipc/common/type_define.h>

namespace uipc::cuda_sanity_check
{
template <int M, int N>
bool need_contact(const core::ContactTabular& contact_table,
                  const Vector<IndexT, M>&    CIdLs,
                  const Vector<IndexT, N>&    CIdRs)
{
    for(auto& L : CIdLs)
    {
        for(auto& R : CIdRs)
        {
            const core::ContactModel& model = contact_table.at(L, R);
            if(!model.is_enabled())
                return false;
        }
    }

    return true;
}

template <int M, int N>
bool need_contact(const core::SubsceneTabular& subscene_table,
                  const Vector<IndexT, M>&     CIdLs,
                  const Vector<IndexT, N>&     CIdRs)
{
    for(auto& L : CIdLs)
    {
        for(auto& R : CIdRs)
        {
            core::SubsceneModel model = subscene_table.at(L, R);
            if(!model.is_enabled())
                return false;
        }
    }

    return true;
}

inline bool need_contact(const core::ContactTabular& contact_table, IndexT CIdL, IndexT CIdR)
{
    core::ContactModel model = contact_table.at(CIdL, CIdR);
    return model.is_enabled();
}

inline bool need_contact(const core::SubsceneTabular& subscene_table, IndexT CIdL, IndexT CIdR)
{
    core::SubsceneModel model = subscene_table.at(CIdL, CIdR);
    return model.is_enabled();
}

template <int N>
inline bool need_contact(const core::ContactTabular& contact_table,
                         IndexT                      CIdL,
                         const Vector<IndexT, N>&    CIdRs)
{
    for(auto& R : CIdRs)
    {
        core::ContactModel model = contact_table.at(CIdL, R);
        if(!model.is_enabled())
            return false;
    }

    return true;
}

template <int N>
inline bool need_contact(const core::SubsceneTabular& subscene_table,
                         IndexT                       CIdL,
                         const Vector<IndexT, N>&     CIdRs)
{
    for(auto& R : CIdRs)
    {
        core::SubsceneModel model = subscene_table.at(CIdL, R);
        if(!model.is_enabled())
            return false;
    }

    return true;
}

inline Float point_dcd_expansion(const Float& d_hat_P)
{
    return d_hat_P;
}

inline Float edge_dcd_expansion(const Float& d_hat_E0, const Float& d_hat_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_E0 == d_hat_E1, "Edge d_hat should be the same");
    }

    return (d_hat_E0 + d_hat_E1) / 2.0;
}

inline Float triangle_dcd_expansion(const Float& d_hat_T0, const Float& d_hat_T1, const Float& d_hat_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_T0 == d_hat_T1 && d_hat_T1 == d_hat_T2,
                    "Triangle d_hat should be the same");
    }

    return (d_hat_T0 + d_hat_T1 + d_hat_T2) / 3.0;
}
}  // namespace uipc::cuda_sanity_check
