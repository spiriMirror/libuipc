#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::joint_limit
{
template <typename T>
UIPC_GENERIC void eval_penalty(const T& x,
                               const T& lower,
                               const T& upper,
                               const T& strength,
                               T&       E,
                               T&       dE_dx,
                               T&       d2E_dx2)
{
    E       = static_cast<T>(0);
    dE_dx   = static_cast<T>(0);
    d2E_dx2 = static_cast<T>(0);

    if(x >= lower && x <= upper)
        return;

    const T w = upper - lower;
    if(w == static_cast<T>(0))
    {
        if(x > upper)
        {
            const T d = x - upper;
            E         = strength * d * d * d;
            dE_dx     = static_cast<T>(3) * strength * d * d;
            d2E_dx2   = static_cast<T>(6) * strength * d;
        }
        else if(x < lower)
        {
            const T d = lower - x;
            E         = strength * d * d * d;
            dE_dx     = -static_cast<T>(3) * strength * d * d;
            d2E_dx2   = static_cast<T>(6) * strength * d;
        }
        return;
    }

    if(x > upper)
    {
        const T r = (x - upper) / w;
        E         = strength * r * r * r;
        dE_dx     = static_cast<T>(3) * strength * r * r / w;
        d2E_dx2   = static_cast<T>(6) * strength * r / (w * w);
    }
    else if(x < lower)
    {
        const T r = (lower - x) / w;
        E         = strength * r * r * r;
        dE_dx     = -static_cast<T>(3) * strength * r * r / w;
        d2E_dx2   = static_cast<T>(6) * strength * r / (w * w);
    }
}
}  // namespace uipc::backend::cuda::joint_limit
