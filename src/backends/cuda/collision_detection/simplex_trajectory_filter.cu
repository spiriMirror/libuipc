#include <collision_detection/simplex_trajectory_filter.h>
#include <cuda_tool/cuda_tool.h>
namespace uipc::backend::cuda
{
namespace
{
    __global__ void SimplexTrajectoryFilter_label_active_vertices_k1_kernel(
        cuda_tool::CBufferView<Vector4i> PTs, cuda_tool::BufferView<IndexT> is_active, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto PT = PTs(i);
        for(int j = 0; j < PT.size(); ++j)
        {
            auto P = PT[j];
            if(is_active(P) == 0)
                cuda_tool::atomic_exch(&is_active(P), 1);
        }
    }

    __global__ void SimplexTrajectoryFilter_label_active_vertices_k2_kernel(
        cuda_tool::CBufferView<Vector4i> EEs, cuda_tool::BufferView<IndexT> is_active, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto EE = EEs(i);
        for(int j = 0; j < EE.size(); ++j)
        {
            auto P = EE[j];
            if(is_active(P) == 0)
                cuda_tool::atomic_exch(&is_active(P), 1);
        }
    }

    __global__ void SimplexTrajectoryFilter_label_active_vertices_k3_kernel(
        cuda_tool::CBufferView<Vector3i> PEs, cuda_tool::BufferView<IndexT> is_active, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto PE = PEs(i);
        for(int j = 0; j < PE.size(); ++j)
        {
            auto P = PE[j];
            if(is_active(P) == 0)
                cuda_tool::atomic_exch(&is_active(P), 1);
        }
    }

    __global__ void SimplexTrajectoryFilter_label_active_vertices_k4_kernel(
        cuda_tool::CBufferView<Vector2i> PPs, cuda_tool::BufferView<IndexT> is_active, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto PP = PPs(i);
        for(int j = 0; j < PP.size(); ++j)
        {
            auto P = PP[j];
            if(is_active(P) == 0)
                cuda_tool::atomic_exch(&is_active(P), 1);
        }
    }
}  // namespace

void SimplexTrajectoryFilter::do_build()
{
    m_impl.global_vertex_manager = require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_body_manager    = require<GlobalBodyManager>();
    const auto constitution =
        world().scene().config().find<std::string>("contact/constitution")->view()[0];
    m_impl.toi_safety_margin       = constitution == "al-ipc" ? 0.001 : 0.1;
    auto& global_trajectory_filter = require<GlobalTrajectoryFilter>();

    BuildInfo info;
    do_build(info);

    global_trajectory_filter.add_filter(this);
}

void SimplexTrajectoryFilter::do_detect(GlobalTrajectoryFilter::DetectInfo& info)
{
    DetectInfo this_info{&m_impl};
    this_info.m_alpha = info.alpha();
    do_detect(this_info);
}

void SimplexTrajectoryFilter::Impl::label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info)
{
    using namespace cuda_tool;

    if(PTs.size() > 0)
    {
        auto k1 = SimplexTrajectoryFilter_label_active_vertices_k1_kernel;
        k1<<<cuda_tool::best_grid_dim((int)PTs.size(), k1), cuda_tool::best_block_dim(k1), 0, nullptr>>>(
            PTs, info.vert_is_active(), (int)PTs.size());
    }

    if(EEs.size() > 0)
    {
        auto k2 = SimplexTrajectoryFilter_label_active_vertices_k2_kernel;
        k2<<<cuda_tool::best_grid_dim((int)EEs.size(), k2), cuda_tool::best_block_dim(k2), 0, nullptr>>>(
            EEs, info.vert_is_active(), (int)EEs.size());
    }

    if(PEs.size() > 0)
    {
        auto k3 = SimplexTrajectoryFilter_label_active_vertices_k3_kernel;
        k3<<<cuda_tool::best_grid_dim((int)PEs.size(), k3), cuda_tool::best_block_dim(k3), 0, nullptr>>>(
            PEs, info.vert_is_active(), (int)PEs.size());
    }

    if(PPs.size() > 0)
    {
        auto k4 = SimplexTrajectoryFilter_label_active_vertices_k4_kernel;
        k4<<<cuda_tool::best_grid_dim((int)PPs.size(), k4), cuda_tool::best_block_dim(k4), 0, nullptr>>>(
            PPs, info.vert_is_active(), (int)PPs.size());
    }
}

void SimplexTrajectoryFilter::do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    FilterActiveInfo this_info{&m_impl};
    do_filter_active(this_info);

    logger::info("SimplexTrajectoryFilter PTs: {}, EEs: {}, PEs: {}, PPs: {}",
                 m_impl.PTs.size(),
                 m_impl.EEs.size(),
                 m_impl.PEs.size(),
                 m_impl.PPs.size());
}

void SimplexTrajectoryFilter::do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info)
{
    FilterTOIInfo this_info{&m_impl};
    this_info.m_alpha = info.alpha();
    this_info.m_toi   = info.toi();
    do_filter_toi(this_info);
}

void SimplexTrajectoryFilter::Impl::record_friction_candidates(
    GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info)
{
    // PT
    loose_resize(friction_PT, PTs.size());
    friction_PT.view().copy_from(PTs);

    // EE
    loose_resize(friction_EE, EEs.size());
    friction_EE.view().copy_from(EEs);

    // PE
    loose_resize(friction_PE, PEs.size());
    friction_PE.view().copy_from(PEs);

    // PP
    loose_resize(friction_PP, PPs.size());
    friction_PP.view().copy_from(PPs);

    logger::info("SimplexTrajectoryFilter Friction PT: {}, EE: {}, PE: {}, PP: {}",
                 friction_PT.size(),
                 friction_EE.size(),
                 friction_PE.size(),
                 friction_PP.size());
}


void SimplexTrajectoryFilter::do_record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info)
{
    m_impl.record_friction_candidates(info);
}

void SimplexTrajectoryFilter::do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info)
{
    m_impl.label_active_vertices(info);
}

Float SimplexTrajectoryFilter::BaseInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}

Float SimplexTrajectoryFilter::BaseInfo::toi_safety_margin() const noexcept
{
    return m_impl->toi_safety_margin;
}

cuda_tool::CBufferView<Float> SimplexTrajectoryFilter::BaseInfo::d_hats() const noexcept
{
    return m_impl->global_vertex_manager->d_hats();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::v2b() const noexcept
{
    return m_impl->global_vertex_manager->body_ids();
}

cuda_tool::CBufferView<Vector3> SimplexTrajectoryFilter::BaseInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}

cuda_tool::CBufferView<Vector3> SimplexTrajectoryFilter::BaseInfo::rest_positions() const noexcept
{
    return m_impl->global_vertex_manager->rest_positions();
}

cuda_tool::CBufferView<Float> SimplexTrajectoryFilter::BaseInfo::thicknesses() const noexcept
{
    return m_impl->global_vertex_manager->thicknesses();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::dimensions() const noexcept
{
    return m_impl->global_vertex_manager->dimensions();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::body_self_collision() const noexcept
{
    return m_impl->global_body_manager->self_collision();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::codim_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->codim_vertices();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}

cuda_tool::CBufferView<Vector2i> SimplexTrajectoryFilter::BaseInfo::surf_edges() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_edges();
}

cuda_tool::CBufferView<Vector3i> SimplexTrajectoryFilter::BaseInfo::surf_triangles() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_triangles();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::contact_element_ids() const noexcept
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

cuda_tool::CBufferView<IndexT> SimplexTrajectoryFilter::BaseInfo::subscene_element_ids() const noexcept
{
    return m_impl->global_vertex_manager->subscene_element_ids();
}

cuda_tool::CBuffer2DView<IndexT> SimplexTrajectoryFilter::BaseInfo::contact_mask_tabular() const noexcept
{
    return m_impl->global_contact_manager->contact_mask_tabular();
}

cuda_tool::CBuffer2DView<IndexT> SimplexTrajectoryFilter::BaseInfo::subscene_mask_tabular() const noexcept
{
    return m_impl->global_contact_manager->subscene_mask_tabular();
}

cuda_tool::CBufferView<Vector4i> SimplexTrajectoryFilter::PTs() const noexcept
{
    return m_impl.PTs;
}

cuda_tool::CBufferView<Vector4i> SimplexTrajectoryFilter::EEs() const noexcept
{
    return m_impl.EEs;
}

cuda_tool::CBufferView<Vector3i> SimplexTrajectoryFilter::PEs() const noexcept
{
    return m_impl.PEs;
}

cuda_tool::CBufferView<Vector2i> SimplexTrajectoryFilter::PPs() const noexcept
{
    return m_impl.PPs;
}

cuda_tool::CBufferView<Vector4i> SimplexTrajectoryFilter::friction_PTs() const noexcept
{
    return m_impl.friction_PT;
}

cuda_tool::CBufferView<Vector4i> SimplexTrajectoryFilter::friction_EEs() const noexcept
{
    return m_impl.friction_EE;
}

cuda_tool::CBufferView<Vector3i> SimplexTrajectoryFilter::friction_PEs() const noexcept
{
    return m_impl.friction_PE;
}


cuda_tool::CBufferView<Vector2i> SimplexTrajectoryFilter::friction_PPs() const noexcept
{
    return m_impl.friction_PP;
}

cuda_tool::CBufferView<Vector3> SimplexTrajectoryFilter::DetectInfo::displacements() const noexcept
{
    return m_impl->global_vertex_manager->displacements();
}

void SimplexTrajectoryFilter::FilterActiveInfo::PTs(cuda_tool::CBufferView<Vector4i> PTs) noexcept
{
    m_impl->PTs = PTs;
}

void SimplexTrajectoryFilter::FilterActiveInfo::EEs(cuda_tool::CBufferView<Vector4i> EEs) noexcept
{
    m_impl->EEs = EEs;
}

void SimplexTrajectoryFilter::FilterActiveInfo::PEs(cuda_tool::CBufferView<Vector3i> PEs) noexcept
{
    m_impl->PEs = PEs;
}

void SimplexTrajectoryFilter::FilterActiveInfo::PPs(cuda_tool::CBufferView<Vector2i> PPs) noexcept
{
    m_impl->PPs = PPs;
}
cuda_tool::VarView<Float> SimplexTrajectoryFilter::FilterTOIInfo::toi() noexcept
{
    return m_toi;
}

void SimplexTrajectoryFilter::do_clear_friction_candidates()
{
    m_impl.friction_PT.resize(0);
    m_impl.friction_EE.resize(0);
    m_impl.friction_PE.resize(0);
    m_impl.friction_PP.resize(0);
}

bool SimplexTrajectoryFilter::Impl::dump(DumpInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_PTs.dump(fmt::format("{}PTs.{}", path, frame), PTs)      //
           && dump_EEs.dump(fmt::format("{}EEs.{}", path, frame), EEs)   //
           && dump_PEs.dump(fmt::format("{}PEs.{}", path, frame), PEs)   //
           && dump_PPs.dump(fmt::format("{}PPs.{}", path, frame), PPs);  //
}

bool SimplexTrajectoryFilter::Impl::try_recover(RecoverInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_PTs.load(fmt::format("{}PTs.{}", path, frame))      //
           && dump_EEs.load(fmt::format("{}EEs.{}", path, frame))   //
           && dump_PEs.load(fmt::format("{}PEs.{}", path, frame))   //
           && dump_PPs.load(fmt::format("{}PPs.{}", path, frame));  //
}

void SimplexTrajectoryFilter::Impl::apply_recover(RecoverInfo& info)
{
    dump_PTs.apply_to(recovered_PT);
    dump_EEs.apply_to(recovered_EE);
    dump_PEs.apply_to(recovered_PE);
    dump_PPs.apply_to(recovered_PP);

    // temporary switch to the recovered PHs, which will be used
    // in the record_friction_candidates() function to recover the friction candidates.
    PTs = recovered_PT.view();
    EEs = recovered_EE.view();
    PEs = recovered_PE.view();
    PPs = recovered_PP.view();
}

void SimplexTrajectoryFilter::Impl::clear_recover(RecoverInfo& info)
{
    dump_PTs.clean_up();
    dump_EEs.clean_up();
    dump_PEs.clean_up();
    dump_PPs.clean_up();
}

bool SimplexTrajectoryFilter::do_dump(DumpInfo& info)
{
    return m_impl.dump(info);
}

bool SimplexTrajectoryFilter::do_try_recover(RecoverInfo& info)
{
    return m_impl.try_recover(info);
}

void SimplexTrajectoryFilter::do_apply_recover(RecoverInfo& info)
{
    m_impl.apply_recover(info);
}

void SimplexTrajectoryFilter::do_clear_recover(RecoverInfo& info)
{
    m_impl.clear_recover(info);
}
}  // namespace uipc::backend::cuda
