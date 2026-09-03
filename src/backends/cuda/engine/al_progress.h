#pragma once

#include <type_define.h>
#include <algorithm>
#include <cmath>

namespace uipc::backend::cuda::details
{
inline IndexT al_effective_k_min(bool semi_implicit_enabled, IndexT configured_k_min)
{
    return semi_implicit_enabled ? std::max(configured_k_min, IndexT{1}) : IndexT{1};
}

inline Float al_update_remaining_weight(Float  remaining_weight,
                                        Float  applied_alpha,
                                        IndexT completed_outer_steps,
                                        IndexT k_min)
{
    if(completed_outer_steps < k_min)
        return remaining_weight;
    return (1.0 - applied_alpha) * remaining_weight;
}

inline bool al_line_search_accepts(Float E0, Float E, bool newton_converged)
{
    if(!std::isfinite(E0) || !std::isfinite(E))
        return false;
    return E <= E0 + 1e-12 || newton_converged;
}
}  // namespace uipc::backend::cuda::details
