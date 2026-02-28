#include <active_set_system/active_set_reporter.h>
#include <active_set_system/global_active_set_manager.h>

namespace uipc::backend::cuda {
    void ActiveSetReporter::do_build() {
        auto& active_set = require<GlobalActiveSetManager>();

        BuildInfo info;
        do_build(info);

        active_set.add_reporter(this);
    }

    void ActiveSetReporter::report_vertex_offset_count(IndexT &offset, IndexT &count) {
        do_report_vertex_offset_count(offset, count);
    }

    void ActiveSetReporter::recover_non_penetrate(NonPenetratePositionsInfo &info) {
        do_recover_non_penetrate(info);
    }
}
