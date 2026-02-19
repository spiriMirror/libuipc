#include <global_geometry/global_vertex_manager.h>
#include <active_set_system/global_active_set_manager.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <muda/cub/device/device_reduce.h>
#include <global_geometry/vertex_reporter.h>
#include <sim_engine.h>
#include <collision_detection/global_trajectory_filter.h>

/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalVertexManager);

void GlobalVertexManager::do_build()
{
    auto d_hat = world().scene().config().find<Float>("contact/d_hat");
    m_impl.default_d_hat = d_hat->view()[0];

    // * Common
    m_impl.global_trajectory_filter = find<GlobalTrajectoryFilter>();
    // * AL-IPC
    m_impl.global_active_set_manager = find<GlobalActiveSetManager>();
}

void GlobalVertexManager::Impl::init()
{
    auto vertex_reporter_view = vertex_reporters.view();

    // 1) Setup index for each vertex reporter

    // ref: https://github.com/spiriMirror/libuipc/issues/271
    // Sort by uid to ensure the order is consistent
    std::ranges::sort(vertex_reporter_view,
                      [](const VertexReporter* l, const VertexReporter* r)
                      { return l->uid() < r->uid(); });
    for(auto&& [i, R] : enumerate(vertex_reporter_view))
        R->m_index = i;

    // 2) Count the number of vertices reported by each reporter
    auto N = vertex_reporter_view.size();
    reporter_vertex_offsets_counts.resize(N);

    span<IndexT> reporter_vertex_counts = reporter_vertex_offsets_counts.counts();

    for(auto&& [i, R] : enumerate(vertex_reporter_view))
    {
        VertexCountInfo info;
        R->report_count(info);
        // get count back
        reporter_vertex_counts[i] = info.m_count;
    }
    reporter_vertex_offsets_counts.scan();
    SizeT total_count = reporter_vertex_offsets_counts.total_count();

    // 3) Initialize buffers for vertex attributes
    coindices.resize(total_count);
    positions.resize(total_count);
    rest_positions.resize(total_count);
    safe_positions.resize(total_count);
    contact_element_ids.resize(total_count, 0);
    subscene_element_ids.resize(total_count, 0);
    thicknesses.resize(total_count, 0.0);
    dimensions.resize(total_count, 3);  // default 3D
    displacements.resize(total_count, Vector3::Zero());
    displacement_norms.resize(total_count, 0.0);
    body_ids.resize(total_count, -1);  // -1 means no care about body id
    d_hats.resize(total_count, default_d_hat);  // use default d_hat if not specified

    // 4) Create the subviews for each attribute_reporter,
    //    so that each reporter can write to its own subview
    for(auto&& [i, R] : enumerate(vertex_reporter_view))
    {
        VertexAttributeInfo attributes{
            this,
            i,
            0  // frame = 0 for initialization
        };
        R->report_attributes(attributes);
    }

    // 5) Initialize previous positions and safe positions
    prev_positions = positions;
    safe_positions = positions;

    // 6) Other initializations
    axis_max_disp = 0.0;
}

void GlobalVertexManager::Impl::update_attributes(SizeT frame)
{
    auto vertex_reporter_view = vertex_reporters.view();

    for(auto&& [i, R] : enumerate(vertex_reporter_view))
    {
        VertexAttributeInfo attributes{this, i, frame};
        R->report_attributes(attributes);
    }
}

void GlobalVertexManager::Impl::rebuild()
{
    UIPC_ASSERT(false, "Not implemented yet");
}

void GlobalVertexManager::add_reporter(VertexReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.vertex_reporters.register_sim_system(*reporter);
}

void GlobalVertexManager::Impl::step_forward(Float alpha)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
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

void GlobalVertexManager::Impl::prepare_AL_CCD() {
    UIPC_ASSERT(global_active_set_manager, "GlobalActiveSetManager not enabled");
    auto non_penetrate_positions = global_active_set_manager->non_penetrate_positions();
    auto& tmp_pos = safe_positions;
    UIPC_ASSERT(non_penetrate_positions.size() == positions.size(), "Non-penetrate size not equal");
    muda::ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(positions.size(),
               [pos      = positions.viewer().name("pos"),
               tmp_pos = tmp_pos.viewer().name("tmp_pos"),
               disp     = displacements.viewer().name("disp"),
               non_penetrate_pos = non_penetrate_positions.viewer().name("non_penetrate_pos")] __device__(int i) mutable {
                   disp(i) = pos(i) - non_penetrate_pos(i);
                   tmp_pos(i) = pos(i);
                   pos(i) = non_penetrate_pos(i);
               });
}

void GlobalVertexManager::Impl::post_AL_CCD() {
    auto& tmp_pos = safe_positions;
    muda::BufferLaunch().copy<Vector3>(positions.view(), std::as_const(tmp_pos).view());
}

void GlobalVertexManager::Impl::recover_non_penetrate() {
    using namespace muda;
    UIPC_ASSERT(global_active_set_manager, "GlobalActiveSetManager not enabled");
    auto non_penetrate_positions = global_active_set_manager->non_penetrate_positions();
    UIPC_ASSERT(non_penetrate_positions.size() == positions.size(), "Non-penetrate size not equal");
    BufferLaunch().copy<Vector3>(positions.view(), non_penetrate_positions);
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

    vertex_bounding_box = AABB{min_pos_host.cast<float>(), max_pos_host.cast<float>()};
    return vertex_bounding_box;
}
}  // namespace uipc::backend::cuda

// Dump & Recover:
namespace uipc::backend::cuda
{
bool GlobalVertexManager::Impl::dump(DumpInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_positions.dump(fmt::format("{}positions.{}", path, frame), positions)  //
           && dump_prev_positions.dump(fmt::format("{}prev_positions.{}", path, frame),
                                       prev_positions);
}

bool GlobalVertexManager::Impl::try_recover(RecoverInfo& info)
{
    auto path = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    return dump_positions.load(fmt::format("{}positions.{}", path, info.frame()))  //
           && dump_prev_positions.load(
               fmt::format("{}prev_positions.{}", path, info.frame()));
}

void GlobalVertexManager::Impl::apply_recover(RecoverInfo& info)
{
    dump_positions.apply_to(positions);
    dump_prev_positions.apply_to(prev_positions);
}

void GlobalVertexManager::Impl::clear_recover(RecoverInfo& info)
{
    dump_positions.clean_up();
    dump_prev_positions.clean_up();
}
}  // namespace uipc::backend::cuda


/*************************************************************************************************
* API Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
void GlobalVertexManager::VertexCountInfo::count(SizeT count) noexcept
{
    m_count = count;
}

void GlobalVertexManager::VertexCountInfo::changeable(bool is_changable) noexcept
{
    m_changable = is_changable;
}

GlobalVertexManager::VertexAttributeInfo::VertexAttributeInfo(Impl* impl, SizeT index, SizeT frame) noexcept
    : m_impl(impl)
    , m_index(index)
    , m_frame(frame)
{
}

muda::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::rest_positions() const noexcept
{
    return m_impl->subview(m_impl->rest_positions, m_index);
}

muda::BufferView<Float> GlobalVertexManager::VertexAttributeInfo::thicknesses() const noexcept
{
    return m_impl->subview(m_impl->thicknesses, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::coindices() const noexcept
{
    return m_impl->subview(m_impl->coindices, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::dimensions() const noexcept
{
    return m_impl->subview(m_impl->dimensions, m_index);
}

muda::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::positions() const noexcept
{
    return m_impl->subview(m_impl->positions, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::contact_element_ids() const noexcept
{
    return m_impl->subview(m_impl->contact_element_ids, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::subscene_element_ids() const noexcept
{
    return m_impl->subview(m_impl->subscene_element_ids, m_index);
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::body_ids() const noexcept
{
    return m_impl->subview(m_impl->body_ids, m_index);
}

muda::BufferView<Float> GlobalVertexManager::VertexAttributeInfo::d_hats() const noexcept
{
    return m_impl->subview(m_impl->d_hats, m_index);  // Assuming d_hats are stored in thicknesses
}

void GlobalVertexManager::VertexAttributeInfo::require_discard_friction() const noexcept
{
    // If the vertex attributes are updated in a way that will ruin the friction computation
    // we need to discard the friction information in the global trajectory filter.
    // ref: https://github.com/spiriMirror/libuipc/issues/303
    if(m_impl->global_trajectory_filter)
        m_impl->global_trajectory_filter->require_discard_friction();
}

SizeT GlobalVertexManager::VertexAttributeInfo::frame() const noexcept
{
    return m_frame;
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

bool GlobalVertexManager::do_dump(DumpInfo& info)
{
    return m_impl.dump(info);
}

bool GlobalVertexManager::do_try_recover(RecoverInfo& info)
{
    return m_impl.try_recover(info);
}

void GlobalVertexManager::do_apply_recover(RecoverInfo& info)
{
    m_impl.apply_recover(info);
}

void GlobalVertexManager::do_clear_recover(RecoverInfo& info)
{
    m_impl.clear_recover(info);
}

void GlobalVertexManager::init()
{
    m_impl.init();
}

void GlobalVertexManager::update_attributes()
{
    m_impl.update_attributes(engine().frame());
}

void GlobalVertexManager::rebuild()
{
    m_impl.rebuild();
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

muda::CBufferView<IndexT> GlobalVertexManager::body_ids() const noexcept
{
    return m_impl.body_ids;
}

muda::CBufferView<Float> GlobalVertexManager::d_hats() const noexcept
{
    return m_impl.d_hats;
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

muda::CBufferView<IndexT> GlobalVertexManager::subscene_element_ids() const noexcept
{
    return m_impl.subscene_element_ids;
}

muda::CBufferView<Vector3> GlobalVertexManager::displacements() const noexcept
{
    return m_impl.displacements;
}

muda::CBufferView<Float> GlobalVertexManager::thicknesses() const noexcept
{
    return m_impl.thicknesses;
}

Float GlobalVertexManager::compute_axis_max_displacement()
{
    return m_impl.compute_axis_max_displacement();
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

void GlobalVertexManager::prepare_AL_CCD() {
    m_impl.prepare_AL_CCD();
}

void GlobalVertexManager::post_AL_CCD() {
    m_impl.post_AL_CCD();
}

void GlobalVertexManager::recover_non_penetrate() {
    m_impl.recover_non_penetrate();
}

muda::CBufferView<IndexT> GlobalVertexManager::dimensions() const noexcept
{
    return m_impl.dimensions;
}

AABB GlobalVertexManager::vertex_bounding_box() const noexcept
{
    return m_impl.vertex_bounding_box;
}
}  // namespace uipc::backend::cuda