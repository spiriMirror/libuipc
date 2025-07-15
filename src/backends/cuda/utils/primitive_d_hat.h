#pragma once
#include <type_define.h>
#include <uipc/common/config.h>
#include <muda/tools/debug_log.h>

namespace uipc::backend::cuda
{
inline MUDA_GENERIC Float point_dcd_expansion(const Float& d_hat_P)
{
    return d_hat_P;
}
/**
 * @brief Edge d_hat
 */
inline MUDA_GENERIC Float edge_dcd_expansion(const Float& d_hat_E0, const Float& d_hat_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        MUDA_ASSERT(d_hat_E0 == d_hat_E1, "Edge d_hat should be the same");
    }

    return (d_hat_E0 + d_hat_E1) / 2.0;
}

/**
 * @brief Triangle d_hat
 */
inline MUDA_GENERIC Float triangle_dcd_expansion(const Float& d_hat_T0,
                                                 const Float& d_hat_T1,
                                                 const Float& d_hat_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        MUDA_ASSERT(d_hat_T0 == d_hat_T1 && d_hat_T1 == d_hat_T2,
                    "Triangle d_hat should be the same");
    }

    return (d_hat_T0 + d_hat_T1 + d_hat_T2) / 3.0;
}

/**
 * @brief Point-Triangle d_hat calculation
 */
inline MUDA_GENERIC Float PT_d_hat(const Float& d_hat_P,
                                   const Float& d_hat_T0,
                                   const Float& d_hat_T1,
                                   const Float& d_hat_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        MUDA_ASSERT(d_hat_T0 == d_hat_T1 && d_hat_T1 == d_hat_T2,
                    "Triangle d_hat should be the same");
    }

    return (d_hat_P + d_hat_T0) / 2.0;
}

/**
 * @brief Edge-Edge d_hat calculation
 */
inline MUDA_GENERIC Float EE_d_hat(const Float& d_hat_Ea0,
                                   const Float& d_hat_Ea1,
                                   const Float& d_hat_Eb0,
                                   const Float& d_hat_Eb1)
{
    if constexpr(RUNTIME_CHECK)
    {
        MUDA_ASSERT(d_hat_Ea0 == d_hat_Ea1 && d_hat_Eb0 == d_hat_Eb1,
                    "Edge d_hat should be the same");
    }

    return (d_hat_Ea0 + d_hat_Eb0) / 2.0;
}

/**
 * @brief Point-Edge d_hat calculation
 */
inline MUDA_GENERIC Float PE_d_hat(const Float& d_hat_P, const Float& d_hat_E0, const Float& d_hat_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        MUDA_ASSERT(d_hat_E0 == d_hat_E1, "Edge d_hat should be the same");
    }

    return (d_hat_P + d_hat_E0) / 2.0;
}

/**
 * @brief Point-Point d_hat calculation
 */
inline MUDA_GENERIC Float PP_d_hat(const Float& d_hat_P0, const Float& d_hat_P1)
{
    return (d_hat_P0 + d_hat_P1) / 2.0;
}
}  // namespace uipc::backend::cuda
