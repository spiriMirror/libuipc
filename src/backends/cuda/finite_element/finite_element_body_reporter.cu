#include <finite_element/finite_element_body_reporter.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FiniteElementBodyReporter_report_attributes_kernel(
        cuda_tool::BufferView<IndexT> coindices, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i) = i;  // just iota
    }
}  // namespace

REGISTER_SIM_SYSTEM(FiniteElementBodyReporter);

void FiniteElementBodyReporter::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
}

void FiniteElementBodyReporter::do_init(InitInfo& info)
{
    // do nothing
}

void FiniteElementBodyReporter::do_report_count(BodyCountInfo& info)
{
    m_impl.report_count(info);
}

void FiniteElementBodyReporter::do_report_attributes(BodyAttributeInfo& info)
{
    m_impl.report_attributes(info);
}

void FiniteElementBodyReporter::Impl::report_count(BodyCountInfo& info)
{
    auto N = finite_element_method->m_impl.h_body_self_collision.size();
    info.count(N);
    info.changeable(false);
}

void FiniteElementBodyReporter::Impl::report_attributes(BodyAttributeInfo& info)
{
    auto k = FiniteElementBodyReporter_report_attributes_kernel;
    int  n = (int)info.coindices().size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.coindices(), n);
    }

    span<const IndexT> self_collision = finite_element_method->m_impl.h_body_self_collision;

    UIPC_ASSERT(self_collision.size() == info.self_collision().size(),
                "Size mismatch in self-collision data, info size: {}, self_collision size: {}",
                info.self_collision().size(),
                self_collision.size());

    info.self_collision().copy_from(self_collision.data());
}
}  // namespace uipc::backend::cuda