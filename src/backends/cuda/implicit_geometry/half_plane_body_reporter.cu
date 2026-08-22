#include <implicit_geometry/half_plane_body_reporter.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void HalfPlaneBodyReporter_report_attributes_kernel(
        cuda_tool::BufferView<IndexT> coindices, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i) = i;  // just iota
    }
}  // namespace

REGISTER_SIM_SYSTEM(HalfPlaneBodyReporter);

void HalfPlaneBodyReporter::do_build(BuildInfo& info)
{
    m_impl.half_plane = &require<HalfPlane>();
}

void HalfPlaneBodyReporter::do_init(InitInfo& info) {}

void HalfPlaneBodyReporter::Impl::report_count(BodyCountInfo& info)
{
    // One position and one normal per half plane body
    // so body_count is equal to the position count.
    auto body_count = half_plane->m_impl.h_positions.size();
    info.count(body_count);
}

void HalfPlaneBodyReporter::Impl::report_attributes(BodyAttributeInfo& info)
{
    using namespace cuda_tool;

    auto k = HalfPlaneBodyReporter_report_attributes_kernel;
    int  n = (int)info.coindices().size();
    if(n > 0)
    {
        k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
            info.coindices().viewer(), n);
    }

    // HalfPlane does not have self-collision, so we fill it with zeros.
    info.self_collision().fill(0);
}

void HalfPlaneBodyReporter::do_report_count(BodyCountInfo& info)
{
    m_impl.report_count(info);
}

void HalfPlaneBodyReporter::do_report_attributes(BodyAttributeInfo& info)
{
    m_impl.report_attributes(info);
}
}  // namespace uipc::backend::cuda
