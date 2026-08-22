#pragma once
#include <type_define.h>
#include <uipc/common/config.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
/**
 * @brief Edge thickness
 */
inline UIPC_GENERIC Float edge_thickness(const Float& thickness_E0, const Float& thickness_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_KERNEL_ASSERT(thickness_E0 == thickness_E1, "Edge thickness should be the same");
    }

    return thickness_E0;
}

/**
 * @brief Triangle thickness
 */
inline UIPC_GENERIC Float triangle_thickness(const Float& thickness_T0,
                                             const Float& thickness_T1,
                                             const Float& thickness_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_KERNEL_ASSERT(thickness_T0 == thickness_T1 && thickness_T1 == thickness_T2,
                           "Triangle thickness should be the same");
    }

    return thickness_T0;
}

/**
 * @brief Point-Triangle thickness calculation
 */
inline UIPC_GENERIC Float PT_thickness(const Float& thickness_P,
                                       const Float& thickness_T0,
                                       const Float& thickness_T1,
                                       const Float& thickness_T2)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_KERNEL_ASSERT(thickness_T0 == thickness_T1 && thickness_T1 == thickness_T2,
                           "Triangle thickness should be the same");
    }

    // return (thickness_P + thickness_T0) / 2.0;
    return thickness_P + thickness_T0;
}

/**
 * @brief Edge-Edge thickness calculation
 */
inline UIPC_GENERIC Float EE_thickness(const Float& thickness_Ea0,
                                       const Float& thickness_Ea1,
                                       const Float& thickness_Eb0,
                                       const Float& thickness_Eb1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_KERNEL_ASSERT(thickness_Ea0 == thickness_Ea1 && thickness_Eb0 == thickness_Eb1,
                           "Edge thickness should be the same");
    }

    // return (thickness_Ea0 + thickness_Eb0) / 2.0;
    return thickness_Ea0 + thickness_Eb0;
}

/**
 * @brief Point-Edge thickness calculation
 */
inline UIPC_GENERIC Float PE_thickness(const Float& thickness_P,
                                       const Float& thickness_E0,
                                       const Float& thickness_E1)
{
    if constexpr(RUNTIME_CHECK)
    {
        UIPC_KERNEL_ASSERT(thickness_E0 == thickness_E1, "Edge thickness should be the same");
    }

    // return (thickness_P + thickness_E0) / 2.0;
    return thickness_P + thickness_E0;
}

/**
 * @brief Point-Point thickness calculation
 */
inline UIPC_GENERIC Float PP_thickness(const Float& thickness_P0, const Float& thickness_P1)
{
    // return (thickness_P0 + thickness_P1) / 2.0;
    return thickness_P0 + thickness_P1;
}

/**
 * @brief the range of d^2, considering the thickness
 */
inline UIPC_GENERIC Vector2 D_range(Float xi, Float d_hat)
{
    auto upper = xi + d_hat;
    auto lower = xi;
    return Vector2{lower * lower, upper * upper};
}

/**
 * @brief check if D is in the active range [D_range.x(), D_range.y())
 */
inline UIPC_GENERIC bool is_active_D(Vector2 D_range, Float D)
{
    return D_range.x() < D && D < D_range.y();
}
}  // namespace uipc::backend::cuda
