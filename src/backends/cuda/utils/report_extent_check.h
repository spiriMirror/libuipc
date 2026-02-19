#pragma once
#include <sim_system.h>
#include <string_view>

namespace uipc::backend::cuda
{
inline void check_report_extent(bool             gradient_only_checked,
                                bool             gradient_only,
                                SizeT            hessian_count,
                                std::string_view name)
{
    UIPC_ASSERT(gradient_only_checked || hessian_count == 0,
                R"(`ReportExtentInfo::gradient_only()` must be called, it seems you forgot to check gradient_only in {}.
Ref: https://github.com/spiriMirror/libuipc/issues/295)",
                name);
    UIPC_ASSERT(!(gradient_only && hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name,
                hessian_count);
}
}  // namespace uipc::backend::cuda
