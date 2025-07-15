#pragma once
#include <uipc/common/type_define.h>

namespace uipc::sanity_check
{
template <int M, int N>
bool need_contact(const ContactTabular&    contact_table,
                  const Vector<IndexT, M>& CIdLs,
                  const Vector<IndexT, N>& CIdRs)
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

inline bool need_contact(const ContactTabular& contact_table, IndexT CIdL, IndexT CIdR)
{
    const core::ContactModel& model = contact_table.at(CIdL, CIdR);
    return model.is_enabled();
}

template <int N>
inline bool need_contact(const ContactTabular&    contact_table,
                         IndexT                   CIdL,
                         const Vector<IndexT, N>& CIdRs)
{
    for(auto& R : CIdRs)
    {
        const core::ContactModel& model = contact_table.at(CIdL, R);

        if(!model.is_enabled())
            return false;
    }

    return true;
}

inline Float point_dcd_expansion(const Float& d_hat_P)
{
    return d_hat_P;
}
/**
 * @brief Edge d_hat
 */
inline Float edge_dcd_expansion(const Float& d_hat_E0, const Float& d_hat_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_E0 == d_hat_E1, "Edge d_hat should be the same");
    }

    return (d_hat_E0 + d_hat_E1) / 2.0;
}

/**
 * @brief Triangle d_hat
 */
inline Float triangle_dcd_expansion(const Float& d_hat_T0, const Float& d_hat_T1, const Float& d_hat_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_T0 == d_hat_T1 && d_hat_T1 == d_hat_T2,
                    "Triangle d_hat should be the same");
    }

    return (d_hat_T0 + d_hat_T1 + d_hat_T2) / 3.0;
}

/**
 * @brief Point-Triangle d_hat calculation
 */
inline Float PT_d_hat(const Float& d_hat_P, const Float& d_hat_T0, const Float& d_hat_T1, const Float& d_hat_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_T0 == d_hat_T1 && d_hat_T1 == d_hat_T2,
                    "Triangle d_hat should be the same");
    }

    return (d_hat_P + d_hat_T0) / 2.0;
}

/**
 * @brief Edge-Edge d_hat calculation
 */
inline Float EE_d_hat(const Float& d_hat_Ea0,
                      const Float& d_hat_Ea1,
                      const Float& d_hat_Eb0,
                      const Float& d_hat_Eb1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_Ea0 == d_hat_Ea1 && d_hat_Eb0 == d_hat_Eb1,
                    "Edge d_hat should be the same");
    }

    return (d_hat_Ea0 + d_hat_Eb0) / 2.0;
}

/**
 * @brief Point-Edge d_hat calculation
 */
inline Float PE_d_hat(const Float& d_hat_P, const Float& d_hat_E0, const Float& d_hat_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_ASSERT(d_hat_E0 == d_hat_E1, "Edge d_hat should be the same");
    }

    return (d_hat_P + d_hat_E0) / 2.0;
}

/**
 * @brief Point-Point d_hat calculation
 */
inline Float PP_d_hat(const Float& d_hat_P0, const Float& d_hat_P1)
{
    return (d_hat_P0 + d_hat_P1) / 2.0;
}
}  // namespace uipc::sanity_check
