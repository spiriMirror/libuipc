#pragma once

#include <type_define.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace uipc::backend::cuda::details
{
inline constexpr Float ALConstraintDecayCutoff  = 1e-2;
inline constexpr Float ALCandidateTOIUpperBound = 1.0 - 1e-6;
inline constexpr Float ALCandidateTOITolerance  = 1e-6;

inline int al_inactive_count_limit(Float decay)
{
    if(!(decay > 0.0 && decay < 1.0) || !std::isfinite(decay))
        return 0;

    const Float raw_limit =
        std::floor(std::log(ALConstraintDecayCutoff) / std::log(decay));
    if(raw_limit >= static_cast<Float>(std::numeric_limits<int>::max() - 1))
        return std::numeric_limits<int>::max() - 1;

    int limit = static_cast<int>(raw_limit);
    limit     = std::max(limit, 0);

    // Correct the result at exact powers (and any libm rounding boundary) so
    // `limit` is precisely the last count whose decay weight is at least 0.01.
    while(limit > 0 && std::pow(decay, limit) < ALConstraintDecayCutoff)
        --limit;
    while(std::pow(decay, limit + 1) >= ALConstraintDecayCutoff)
        ++limit;
    return limit;
}

inline UIPC_GENERIC Float al_nonnegative_candidate_toi(Float toi)
{
    return toi < 0.0 ? 0.0 : toi;
}

inline UIPC_GENERIC bool al_candidate_has_collision(Float toi)
{
    return toi == toi && al_nonnegative_candidate_toi(toi) < ALCandidateTOIUpperBound;
}

inline UIPC_GENERIC bool al_keep_new_candidate(Float toi, Float max_incident_min_toi)
{
    if(!al_candidate_has_collision(toi) || max_incident_min_toi != max_incident_min_toi)
        return false;

    const Float candidate_toi = al_nonnegative_candidate_toi(toi);
    return candidate_toi <= max_incident_min_toi + ALCandidateTOITolerance;
}

inline UIPC_DEVICE void al_atomic_min_candidate_toi(Float* address, Float toi)
{
    static_assert(sizeof(Float) == sizeof(unsigned long long));
    if(toi != toi)
        return;

    toi        = al_nonnegative_candidate_toi(toi);
    auto* bits = reinterpret_cast<unsigned long long*>(address);
    auto  old  = atomicCAS(bits, 0ULL, 0ULL);
    while(toi < __longlong_as_double(static_cast<long long>(old)))
    {
        const auto assumed = old;
        old                = atomicCAS(bits,
                        assumed,
                        static_cast<unsigned long long>(__double_as_longlong(toi)));
        if(old == assumed)
            break;
    }
}
}  // namespace uipc::backend::cuda::details
