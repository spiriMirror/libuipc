#pragma once
#include <type_define.h>
#include <uipc/common/config.h>
#include <muda/tools/debug_log.h>

namespace uipc::backend::cuda::joint_limit
{
namespace detail
{
template <bool ComputeE, bool ComputeDerivatives, typename T>
UIPC_GENERIC void eval_penalty_impl(const T& x,
                                    const T& lower,
                                    const T& upper,
                                    const T& strength,
                                    T&       E,
                                    T&       dE_dx,
                                    T&       d2E_dx2)
{
    const T zero = static_cast<T>(0);
    const T one  = static_cast<T>(1);
    const T three = static_cast<T>(3);
    const T six   = static_cast<T>(6);

    if constexpr(RUNTIME_CHECK)
    {
        MUDA_ASSERT(lower <= upper,
                    "joint_limit::eval_penalty requires lower <= upper, but got lower=%f, upper=%f",
                    static_cast<double>(lower),
                    static_cast<double>(upper));
    }

    if constexpr(ComputeE)
        E = zero;
    if constexpr(ComputeDerivatives)
    {
        dE_dx   = zero;
        d2E_dx2 = zero;
    }

    if(x >= lower && x <= upper)
        return;

    const T w = upper - lower;
    if(w == zero)
    {
        if(x > upper)
        {
            const T d = x - upper;
            if constexpr(ComputeE)
                E = strength * d * d * d;
            if constexpr(ComputeDerivatives)
            {
                dE_dx   = three * strength * d * d;
                d2E_dx2 = six * strength * d;
            }
        }
        else
        {
            const T d = lower - x;
            if constexpr(ComputeE)
                E = strength * d * d * d;
            if constexpr(ComputeDerivatives)
            {
                dE_dx   = -three * strength * d * d;
                d2E_dx2 = six * strength * d;
            }
        }
        return;
    }

    const T inv_w = one / w;
    if(x > upper)
    {
        const T r = (x - upper) * inv_w;
        if constexpr(ComputeE)
            E = strength * r * r * r;
        if constexpr(ComputeDerivatives)
        {
            dE_dx   = three * strength * r * r * inv_w;
            d2E_dx2 = six * strength * r * inv_w * inv_w;
        }
    }
    else
    {
        const T r = (lower - x) * inv_w;
        if constexpr(ComputeE)
            E = strength * r * r * r;
        if constexpr(ComputeDerivatives)
        {
            dE_dx   = -three * strength * r * r * inv_w;
            d2E_dx2 = six * strength * r * inv_w * inv_w;
        }
    }
}
}  // namespace detail

template <typename T>
UIPC_GENERIC void eval_penalty(const T& x,
                               const T& lower,
                               const T& upper,
                               const T& strength,
                               T&       E,
                               T&       dE_dx,
                               T&       d2E_dx2)
{
    detail::eval_penalty_impl<true, true>(
        x, lower, upper, strength, E, dE_dx, d2E_dx2);
}

template <typename T>
UIPC_GENERIC T eval_penalty_energy(const T& x,
                                   const T& lower,
                                   const T& upper,
                                   const T& strength)
{
    T E       = static_cast<T>(0);
    T dE_dx   = static_cast<T>(0);
    T d2E_dx2 = static_cast<T>(0);
    detail::eval_penalty_impl<true, false>(
        x, lower, upper, strength, E, dE_dx, d2E_dx2);
    return E;
}

template <typename T>
UIPC_GENERIC void eval_penalty_derivatives(const T& x,
                                           const T& lower,
                                           const T& upper,
                                           const T& strength,
                                           T&       dE_dx,
                                           T&       d2E_dx2)
{
    T E = static_cast<T>(0);
    detail::eval_penalty_impl<false, true>(
        x, lower, upper, strength, E, dE_dx, d2E_dx2);
}
}  // namespace uipc::backend::cuda::joint_limit
