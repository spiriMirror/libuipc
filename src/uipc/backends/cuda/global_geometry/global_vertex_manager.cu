#include <global_geometry/global_vertex_manager.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <muda/cub/device/device_reduce.h>
#include <global_geometry/vertex_reporter.h>

/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
void GlobalVertexManager::Impl::init_vertex_info()
{
    vertex_reporters.init();
    auto vertex_reporter_view = vertex_reporters.view();
    for(auto&& [i, R] : enumerate(vertex_reporter_view))
        R->m_index = i;

    auto N = vertex_reporter_view.size();
    reporter_vertex_counts.resize(N);
    reporter_vertex_offsets.resize(N);

    for(auto&& [i, R] : enumerate(vertex_reporter_view))
    {
        VertexCountInfo info;
        R->report_count(info);
        // get count back
        reporter_vertex_counts[i] = info.m_count;
    }

    std::exclusive_scan(reporter_vertex_counts.begin(),
                        reporter_vertex_counts.end(),
                        reporter_vertex_offsets.begin(),
                        0);

    auto total_count = reporter_vertex_offsets.back() + reporter_vertex_counts.back();

    // resize the global coindices buffer
    coindices.resize(total_count);
    positions.resize(total_count);
    rest_positions.resize(total_count);
    safe_positions.resize(total_count);
    contact_element_ids.resize(total_count, 0);
    displacements.resize(total_count, Vector3::Zero());
    displacement_norms.resize(total_count, 0.0);

    // create the subviews for each attribute_reporter
    for(auto&& [i, R] : enumerate(vertex_reporter_view))
    {
        VertexAttributeInfo attributes{this, i};
        R->report_attributes(attributes);
    }

    // TODO: now just copy at the first time
    // latter, we need to check if user fill rest_positions
    // if not, then copy, otherwise, just use it
    rest_positions = positions;
    prev_positions = positions;
}

void GlobalVertexManager::Impl::rebuild_vertex_info()
{
    UIPC_ASSERT(false, "Not implemented yet");
}

void GlobalVertexManager::add_reporter(VertexReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.vertex_reporters.register_subsystem(*reporter);
}

void GlobalVertexManager::Impl::step_forward(Float alpha)
{
    using namespace muda;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(positions.size(),
               [pos      = positions.viewer().name("pos"),
                safe_pos = safe_positions.viewer().name("safe_pos"),
                disp     = displacements.viewer().name("disp"),
                alpha    = alpha] __device__(int i) mutable
               { pos(i) = safe_pos(i) + alpha * disp(i); });
}

void GlobalVertexManager::Impl::collect_vertex_displacements()
{
    for(auto&& [i, R] : enumerate(vertex_reporters.view()))
    {
        VertexDisplacementInfo vd{this, i};
        R->report_displacements(vd);
    }
}

void GlobalVertexManager::Impl::record_prev_positions()
{
    using namespace muda;
    BufferLaunch().copy<Vector3>(prev_positions.view(), std::as_const(positions).view());
}

void GlobalVertexManager::Impl::record_start_point()
{
    using namespace muda;
    BufferLaunch().copy<Vector3>(safe_positions.view(), std::as_const(positions).view());
}

Float GlobalVertexManager::Impl::compute_axis_max_displacement()
{
    muda::DeviceReduce().Reduce((Float*)displacements.data(),
                                axis_max_disp.data(),
                                displacements.size() * 3,
                                [] CUB_RUNTIME_FUNCTION(const Float& L, const Float& R)
                                {
                                    auto absL = std::abs(L);
                                    auto absR = std::abs(R);
                                    return absL > absR ? absL : absR;
                                },
                                0.0);
    return axis_max_disp;
}

Float GlobalVertexManager::Impl::compute_max_displacement_norm()
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(displacements.size(),
               [disps = displacements.cviewer().name("disp"),
                disp_norms = displacement_norms.viewer().name("disp_norm")] __device__(int i) mutable
               {
                   auto d        = disps(i).norm();
                   disp_norms(i) = d;
               });

    Float max_float = std::numeric_limits<Float>::max();

    DeviceReduce().Max(displacement_norms.data(),
                       max_disp_norm.data(),
                       displacement_norms.size());

    return max_disp_norm;
}

AABB GlobalVertexManager::Impl::compute_vertex_bounding_box()
{
    Float max_float = std::numeric_limits<Float>::max();
    muda::DeviceReduce()
        .Reduce(
            positions.data(),
            min_pos.data(),
            positions.size(),
            [] CUB_RUNTIME_FUNCTION(const Vector3& L, const Vector3& R) -> Vector3
            { return L.cwiseMin(R); },
            Vector3{max_float, max_float, max_float})
        .Reduce(
            positions.data(),
            max_pos.data(),
            positions.size(),
            [] CUB_RUNTIME_FUNCTION(const Vector3& L, const Vector3& R) -> Vector3
            { return L.cwiseMax(R); },
            Vector3{-max_float, -max_float, -max_float});

    Vector3 min_pos_host, max_pos_host;
    min_pos_host = min_pos;
    max_pos_host = max_pos;

    vertex_bounding_box = AABB{min_pos_host, max_pos_host};
    return vertex_bounding_box;
}
}  // namespace uipc::backend::cuda


/*************************************************************************************************
* API Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalVertexManager);

void GlobalVertexManager::VertexCountInfo::count(SizeT count) noexcept
{
    m_count = count;
}

void GlobalVertexManager::VertexCountInfo::changable(bool is_changable) noexcept
{
    m_changable = is_changable;
}

GlobalVertexManager::VertexAttributeInfo::VertexAttributeInfo(Impl* impl, SizeT index) noexcept
    : m_impl(impl)
    , m_index(index)
{
}

muda::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::rest_positions() const noexcept
{
    return m_impl->subview(m_impl->rest_positions, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::coindices() const noexcept
{
    return m_impl->subview(m_impl->coindices, m_index);
}

muda::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::positions() const noexcept
{
    return m_impl->subview(m_impl->positions, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::contact_element_ids() const noexcept
{
    return m_impl->subview(m_impl->contact_element_ids, m_index);
}

GlobalVertexManager::VertexDisplacementInfo::VertexDisplacementInfo(Impl* impl, SizeT index) noexcept
    : m_impl(impl)
    , m_index(index)
{
}

muda::BufferView<Vector3> GlobalVertexManager::VertexDisplacementInfo::displacements() const noexcept
{
    return m_impl->subview(m_impl->displacements, m_index);
}

muda::CBufferView<IndexT> GlobalVertexManager::VertexDisplacementInfo::coindices() const noexcept
{
    return m_impl->subview(m_impl->coindices, m_index);
}

void GlobalVertexManager::do_build() {}

void GlobalVertexManager::init_vertex_info()
{
    m_impl.init_vertex_info();
}

void GlobalVertexManager::rebuild_vertex_info()
{
    m_impl.rebuild_vertex_info();
}

void GlobalVertexManager::record_prev_positions()
{
    m_impl.record_prev_positions();
}

void GlobalVertexManager::collect_vertex_displacements()
{
    m_impl.collect_vertex_displacements();
}

muda::CBufferView<IndexT> GlobalVertexManager::coindices() const noexcept
{
    return m_impl.coindices;
}

muda::CBufferView<Vector3> GlobalVertexManager::positions() const noexcept
{
    return m_impl.positions;
}

muda::CBufferView<Vector3> GlobalVertexManager::prev_positions() const noexcept
{
    return m_impl.prev_positions;
}

muda::CBufferView<Vector3> GlobalVertexManager::rest_positions() const noexcept
{
    return m_impl.rest_positions;
}

muda::CBufferView<Vector3> GlobalVertexManager::safe_positions() const noexcept
{
    return m_impl.safe_positions;
}

muda::CBufferView<IndexT> GlobalVertexManager::contact_element_ids() const noexcept
{
    return m_impl.contact_element_ids;
}

muda::CBufferView<Vector3> GlobalVertexManager::displacements() const noexcept
{
    return m_impl.displacements;
}

Float GlobalVertexManager::compute_axis_max_displacement()
{
    return m_impl.compute_axis_max_displacement();
}

Float GlobalVertexManager::compute_max_displacement_norm()
{
    return m_impl.compute_max_displacement_norm();
}

AABB GlobalVertexManager::compute_vertex_bounding_box()
{
    return m_impl.compute_vertex_bounding_box();
}

void GlobalVertexManager::step_forward(Float alpha)
{
    m_impl.step_forward(alpha);
}

void GlobalVertexManager::record_start_point()
{
    m_impl.record_start_point();
}

AABB GlobalVertexManager::vertex_bounding_box() const noexcept
{
    return m_impl.vertex_bounding_box;
}
}  // namespace uipc::backend::cuda