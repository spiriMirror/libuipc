#include <finite_element/fem_active_set_reporter.h>

namespace uipc::backend::cuda {

void FEMActiveSetReporter::Impl::recover_non_penetrate(NonPenetratePositionsInfo &info) {
    UIPC_ASSERT(fem().xs.size() == info.non_penetrate_positions().size(), "Vertex size not matched");
    fem().xs.view().copy_from(info.non_penetrate_positions());
}

void FEMActiveSetReporter::do_build(BuildInfo &info) {
    m_impl.vertex_reporter = require<FiniteElementVertexReporter>();
    m_impl.finite_element_method = require<FiniteElementMethod>();
}

void FEMActiveSetReporter::do_report_vertex_offset_count(IndexT &offset, IndexT &count) {
    offset = m_impl.vertex_reporter->vertex_offset();
    count = m_impl.vertex_reporter->vertex_count();
}

void FEMActiveSetReporter::do_recover_non_penetrate(NonPenetratePositionsInfo &info) {
    m_impl.recover_non_penetrate(info);
}

REGISTER_SIM_SYSTEM(FEMActiveSetReporter);

}
