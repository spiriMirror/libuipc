#include <global_geometry/global_vertex_manager.h>
#include <active_set_system/global_active_set_manager.h>
#include <joint_dof_system/global_joint_dof_manager.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <global_geometry/vertex_reporter.h>
#include <collision_detection/global_trajectory_filter.h>
#include <sim_engine.h>

/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
namespace
{
    __global__ void GlobalVertexManager_step_forward_kernel(
        cuda_tool::BufferView<Vector3> pos,
        cuda_tool::BufferView<Vector3> safe_pos,
        cuda_tool::BufferView<Vector3> disp,
        Float                          alpha,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        pos(i) = safe_pos(i) + alpha * disp(i);
    }

    __global__ void GlobalVertexManager_setup_ccd_kernel(
        cuda_tool::BufferView<Vector3>  pos,
        cuda_tool::BufferView<Vector3>  tmp_pos,
        cuda_tool::BufferView<Vector3>  disp,
        cuda_tool::CBufferView<Vector3> base_pos,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        disp(i)    = pos(i) - base_pos(i);
        tmp_pos(i) = pos(i);
        pos(i)     = base_pos(i);
    }

    struct GlobalVertexManager_MaxAbsOp
    {
        CUB_RUNTIME_FUNCTION Float operator()(const Float& L, const Float& R) const
        {
            auto absL = std::abs(L);
            auto absR = std::abs(R);
            return absL > absR ? absL : absR;
        }
    };

    struct GlobalVertexManager_CwiseMinOp
    {
        CUB_RUNTIME_FUNCTION Vector3 operator()(const Vector3& L, const Vector3& R) const
        {
            return L.cwiseMin(R);
        }
    };

    struct GlobalVertexManager_CwiseMaxOp
    {
        CUB_RUNTIME_FUNCTION Vector3 operator()(const Vector3& L, const Vector3& R) const
        {
            return L.cwiseMax(R);
        }
    };

    __global__ void GlobalVertexManager_propagate_relative_d_hat_kernel(
        cuda_tool::BufferView<Float> d_hats, Float old_default, Float new_d_hat, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // only rewrite entries still holding the absolute contact/d_hat default;
        // per-geometry meta d_hat values are left untouched
        if(d_hats(i) == old_default)
            d_hats(i) = new_d_hat;
    }
}  // namespace

REGISTER_SIM_SYSTEM(GlobalVertexManager);

void GlobalVertexManager::do_build()
{
    auto d_hat = world().scene().config().find<Float>("contact/d_hat");
    m_impl.default_d_hat = d_hat->view()[0];

    auto d_hat_relative = world().scene().config().find<Float>("contact/d_hat_relative");
    m_impl.d_hat_relative = d_hat_relative ? d_hat_relative->view()[0] : 0.0;

    m_impl.global_trajectory_filter  = find<GlobalTrajectoryFilter>();
    m_impl.global_active_set_manager = find<GlobalActiveSetManager>();
    m_impl.global_joint_dof_manager  = find<GlobalJointDofManager>();
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

    // 7) Scene diagonal of the rest configuration, for scene-relative
    //    parameter adaptation (contact/d_hat_relative etc.)
    {
        auto box       = compute_vertex_bounding_box();
        scene_diagonal = (box.max() - box.min()).norm();
        logger::info("Scene diagonal: {}", scene_diagonal);
    }

    // 8) Propagate the scene-relative d_hat (contact/d_hat_relative, Stiff-GIPC
    //    convention) into the per-vertex d_hats buffer: entries still holding
    //    the absolute contact/d_hat default are replaced, per-geometry meta
    //    d_hat values are untouched. Without this, the relative override only
    //    affected GlobalContactManager's scalar (CFL/logging) while the contact
    //    kernels kept using the absolute default.
    if(d_hat_relative > 0.0)
    {
        Float rel_d_hat = d_hat_relative * scene_diagonal;
        logger::info("Per-vertex d_hat (relative): {} = {} x scene_diagonal({})",
                     rel_d_hat,
                     d_hat_relative,
                     scene_diagonal);
        auto k = GlobalVertexManager_propagate_relative_d_hat_kernel;
        int  n = (int)d_hats.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                d_hats.view(), default_d_hat, rel_d_hat, n);
        default_d_hat = rel_d_hat;
    }
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
    using namespace cuda_tool;

    auto k = GlobalVertexManager_step_forward_kernel;
    int  n = (int)positions.size();
    if(n > 0)
    {
        k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
            positions.view(), safe_positions.view(), displacements.view(), alpha, n);
    }
}

void GlobalVertexManager::Impl::collect_vertex_displacements()
{
    for(auto&& [i, R] : enumerate(vertex_reporters.view()))
    {
        VertexDisplacementInfo vd{this, i};
        R->report_displacements(vd);
    }
}

void GlobalVertexManager::Impl::setup_ccd(cuda_tool::CBufferView<Vector3> base_positions)
{
    auto& tmp_pos = safe_positions;
    UIPC_ASSERT(base_positions.size() == positions.size(),
                "Base positions size not equal to vertex count");
    auto k = GlobalVertexManager_setup_ccd_kernel;
    int  n = (int)positions.size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            positions.view(), tmp_pos.view(), displacements.view(), base_positions, n);
    }
}

void GlobalVertexManager::Impl::restore_ccd()
{
    cuda_tool::BufferLaunch().copy<Vector3>(positions.view(),
                                            std::as_const(safe_positions).view());
}

void GlobalVertexManager::Impl::overwrite_positions(cuda_tool::CBufferView<Vector3> src)
{
    UIPC_ASSERT(src.size() == positions.size(), "Source size not equal to vertex count");
    cuda_tool::BufferLaunch().copy<Vector3>(positions.view(), src);
}

void GlobalVertexManager::VertexAttributeInfo::require_discard_friction() const noexcept
{
    // If the vertex attributes are updated in a way that will ruin the friction computation
    // (e.g. geometry reset, topology change), all friction systems must discard their
    // stale candidates before recording new ones at the start of the next frame.
    // Without discarding, incorrect frictional forces would be applied.
    // ref: https://github.com/spiriMirror/libuipc/issues/303
    if(auto& gtf = m_impl->global_trajectory_filter)
        gtf->require_discard_friction();
    if(auto& gasm = m_impl->global_active_set_manager)
        gasm->require_discard_friction();
}

void GlobalVertexManager::Impl::record_prev_positions()
{
    using namespace cuda_tool;
    BufferLaunch().copy<Vector3>(prev_positions.view(), std::as_const(positions).view());
}

void GlobalVertexManager::Impl::record_start_point()
{
    using namespace cuda_tool;
    BufferLaunch().copy<Vector3>(safe_positions.view(), std::as_const(positions).view());
}

Float GlobalVertexManager::Impl::compute_axis_max_displacement()
{
    cuda_tool::DeviceReduce().Reduce((Float*)displacements.data(),
                                     axis_max_disp.data(),
                                     displacements.size() * 3,
                                     GlobalVertexManager_MaxAbsOp{},
                                     0.0);
    return axis_max_disp;
}

AABB GlobalVertexManager::Impl::compute_vertex_bounding_box()
{
    Float max_float = std::numeric_limits<Float>::max();
    cuda_tool::DeviceReduce()
        .Reduce(positions.data(),
                min_pos.data(),
                positions.size(),
                GlobalVertexManager_CwiseMinOp{},
                Vector3{max_float, max_float, max_float})
        .Reduce(positions.data(),
                max_pos.data(),
                positions.size(),
                GlobalVertexManager_CwiseMaxOp{},
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

cuda_tool::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::rest_positions() const noexcept
{
    return m_impl->subview(m_impl->rest_positions, m_index);
}

cuda_tool::BufferView<Float> GlobalVertexManager::VertexAttributeInfo::thicknesses() const noexcept
{
    return m_impl->subview(m_impl->thicknesses, m_index);
}

cuda_tool::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::coindices() const noexcept
{
    return m_impl->subview(m_impl->coindices, m_index);
}

cuda_tool::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::dimensions() const noexcept
{
    return m_impl->subview(m_impl->dimensions, m_index);
}

cuda_tool::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::positions() const noexcept
{
    return m_impl->subview(m_impl->positions, m_index);
}

cuda_tool::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::contact_element_ids() const noexcept
{
    return m_impl->subview(m_impl->contact_element_ids, m_index);
}

cuda_tool::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::subscene_element_ids() const noexcept
{
    return m_impl->subview(m_impl->subscene_element_ids, m_index);
}

cuda_tool::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::body_ids() const noexcept
{
    return m_impl->subview(m_impl->body_ids, m_index);
}

cuda_tool::BufferView<Float> GlobalVertexManager::VertexAttributeInfo::d_hats() const noexcept
{
    return m_impl->subview(m_impl->d_hats, m_index);  // Assuming d_hats are stored in thicknesses
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

cuda_tool::BufferView<Vector3> GlobalVertexManager::VertexDisplacementInfo::displacements() const noexcept
{
    return m_impl->subview(m_impl->displacements, m_index);
}

cuda_tool::CBufferView<IndexT> GlobalVertexManager::VertexDisplacementInfo::coindices() const noexcept
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

cuda_tool::CBufferView<IndexT> GlobalVertexManager::coindices() const noexcept
{
    return m_impl.coindices;
}

cuda_tool::CBufferView<IndexT> GlobalVertexManager::body_ids() const noexcept
{
    return m_impl.body_ids;
}

cuda_tool::CBufferView<Float> GlobalVertexManager::d_hats() const noexcept
{
    return m_impl.d_hats;
}

cuda_tool::CBufferView<Vector3> GlobalVertexManager::positions() const noexcept
{
    return m_impl.positions;
}

cuda_tool::CBufferView<Vector3> GlobalVertexManager::prev_positions() const noexcept
{
    return m_impl.prev_positions;
}

cuda_tool::CBufferView<Vector3> GlobalVertexManager::rest_positions() const noexcept
{
    return m_impl.rest_positions;
}

cuda_tool::CBufferView<Vector3> GlobalVertexManager::safe_positions() const noexcept
{
    return m_impl.safe_positions;
}

cuda_tool::CBufferView<IndexT> GlobalVertexManager::contact_element_ids() const noexcept
{
    return m_impl.contact_element_ids;
}

cuda_tool::CBufferView<IndexT> GlobalVertexManager::subscene_element_ids() const noexcept
{
    return m_impl.subscene_element_ids;
}

cuda_tool::CBufferView<Vector3> GlobalVertexManager::displacements() const noexcept
{
    return m_impl.displacements;
}

cuda_tool::CBufferView<Float> GlobalVertexManager::thicknesses() const noexcept
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

Float GlobalVertexManager::scene_diagonal() const noexcept
{
    return m_impl.scene_diagonal;
}

void GlobalVertexManager::step_forward(Float alpha)
{
    m_impl.step_forward(alpha);
}

void GlobalVertexManager::record_start_point()
{
    m_impl.record_start_point();
}

void GlobalVertexManager::setup_ccd(cuda_tool::CBufferView<Vector3> base_positions)
{
    m_impl.setup_ccd(base_positions);
}

void GlobalVertexManager::restore_ccd()
{
    m_impl.restore_ccd();
}

void GlobalVertexManager::overwrite_positions(cuda_tool::CBufferView<Vector3> src)
{
    m_impl.overwrite_positions(src);
}

cuda_tool::CBufferView<IndexT> GlobalVertexManager::dimensions() const noexcept
{
    return m_impl.dimensions;
}

AABB GlobalVertexManager::vertex_bounding_box() const noexcept
{
    return m_impl.vertex_bounding_box;
}
}  // namespace uipc::backend::cuda