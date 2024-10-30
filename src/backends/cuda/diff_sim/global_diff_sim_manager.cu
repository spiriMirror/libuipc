#include <diff_sim/global_diff_sim_manager.h>
#include <diff_sim/diff_dof_reporter.h>
#include <diff_sim/diff_parm_reporter.h>
#include <linear_system/global_linear_system.h>
#include <sim_engine.h>
#include <utils/offset_count_collection.h>
#include <sim_engine.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::GlobalDiffSimManager>
{
  public:
    static U<cuda::GlobalDiffSimManager> create(SimEngine& engine)
    {
        auto scene = dynamic_cast<SimEngine&>(engine).world().scene();
        if(!scene.info()["diff_sim"]["enable"].get<bool>())
        {
            return nullptr;
        }
        return uipc::make_unique<cuda::GlobalDiffSimManager>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
namespace detail
{
    void build_coo_matrix(muda::LinearSystemContext&           ctx,
                          muda::DeviceCOOMatrix<Float>&        total_coo,
                          muda::DeviceTripletMatrix<Float, 1>& total_triplet,
                          muda::DeviceTripletMatrix<Float, 1>& local_triplet)
    {
        using namespace muda;

        // 1) reshape the total_coo and total_triplet
        auto M = local_triplet.rows();
        auto N = local_triplet.cols();
        total_coo.reshape(M, N);
        total_triplet.reshape(M, N);

        // 2) append the local_triplet to the total_triplet
        auto new_triplet_count = total_coo.non_zeros() + local_triplet.triplet_count();
        total_triplet.resize_triplets(new_triplet_count);
        auto total_triplet_view = total_triplet.view();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(total_coo.non_zeros(),
                   [total_coo     = total_coo.cviewer().name("total_coo"),
                    total_triplet = total_triplet_view
                                        .subview(0, total_coo.non_zeros())  // front
                                        .viewer()
                                        .name("total_triplet")] __device__(int I) mutable
                   {
                       auto&& [i, j, V] = total_coo(I);
                       total_triplet(I).write(i, j, V);
                   });

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(local_triplet.triplet_count(),
                   [local_triplet = local_triplet.cviewer().name("local_triplet"),
                    total_triplet = total_triplet_view
                                        .subview(total_coo.non_zeros(),
                                                 local_triplet.triplet_count())  // back
                                        .viewer()
                                        .name("total_triplet")] __device__(int I) mutable
                   {
                       auto&& [i, j, V] = local_triplet(I);
                       total_triplet(I).write(i, j, V);
                   });

        // 3) convert the total_triplet to total_coo
        ctx.convert(total_triplet, total_coo);
    }

    void copy_to_host(const muda::DeviceCOOMatrix<Float>& total_coo,
                      GlobalDiffSimManager::SparseCOO&    host_coo)
    {
        // copy row_inides, col_indices, values to host_coo

        host_coo.row_indices.resize(total_coo.row_indices().size());
        total_coo.row_indices().copy_to(host_coo.row_indices.data());

        host_coo.col_indices.resize(total_coo.col_indices().size());
        total_coo.col_indices().copy_to(host_coo.col_indices.data());

        host_coo.values.resize(total_coo.values().size());
        total_coo.values().copy_to(host_coo.values.data());

        host_coo.shape = {total_coo.rows(), total_coo.cols()};
    }
}  // namespace detail
// namespace detail
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalDiffSimManager);

void GlobalDiffSimManager::do_build()
{
    m_impl.global_linear_system = &require<GlobalLinearSystem>();
    m_impl.sim_engine           = &engine();

    on_write_scene([&] { m_impl.write_scene(world()); });
}

muda::LinearSystemContext& GlobalDiffSimManager::Impl::ctx()
{
    return global_linear_system->m_impl.ctx;
}

void GlobalDiffSimManager::Impl::init(WorldVisitor& world)
{
    auto& diff_sim   = world.scene().diff_sim();
    auto  parm_view  = diff_sim.parameters().view();
    total_parm_count = parm_view.size();
    dof_offsets.reserve(1024);
    dof_counts.reserve(1024);
    total_coo_pGpP.reshape(0, 0);
    total_coo_H.reshape(0, 0);

    // 1) Copy the parameters to the device
    parameters.resize(total_parm_count);
    parameters.view().copy_from(parm_view.data());


    // 2) Init the diff_parm_reporters
    {
        auto diff_parm_reporter_view = diff_parm_reporters.view();
        for(auto&& [i, R] : enumerate(diff_parm_reporter_view))
        {
            R->m_index = i;
        }

        diff_parm_triplet_offset_count.resize(diff_parm_reporter_view.size());
    }

    // 3) Init the diff_dof_reporters
    {
        auto diff_dof_reporter_view = diff_dof_reporters.view();
        for(auto&& [i, R] : enumerate(diff_dof_reporter_view))
        {
            R->m_index = i;
        }

        diff_dof_triplet_offset_count.resize(diff_dof_reporter_view.size());
    }
}

void GlobalDiffSimManager::Impl::update()
{
    auto& diff_sim = sim_engine->world().scene().diff_sim();

    if(!diff_sim.need_backend_broadcast())
        return;

    // 1) Copy the new parameters to the device

    auto parm_view = diff_sim.parameters().view();
    parameters.view().copy_from(parm_view.data());

    // 2) Update the diff_parm_reporters
    auto diff_parm_reporter_view = diff_parm_reporters.view();
    for(auto&& R : diff_parm_reporter_view)
    {
        DiffParmUpdateInfo info{this};
        R->update_diff_parm(info);
    }

    diff_sim.need_backend_broadcast(false);
}

void GlobalDiffSimManager::Impl::assemble()
{
    // 0) Prepare the frame dof count
    auto current_frame_dof_offset = total_frame_dof_count;
    current_frame_dof_count       = global_linear_system->dof_count();
    total_frame_dof_count += current_frame_dof_count;
    dof_offsets.push_back(current_frame_dof_offset);
    dof_counts.push_back(current_frame_dof_count);


    // 1) Assemble the pGpH Triplets
    {
        auto diff_parm_reporter_view  = diff_parm_reporters.view();
        auto diff_parm_triplet_counts = diff_parm_triplet_offset_count.counts();

        for(auto&& [i, R] : enumerate(diff_parm_reporter_view))
        {
            DiffParmExtentInfo info{this, R->m_index};
            R->report_extent(info);

            diff_parm_triplet_counts[i] = info.m_triplet_count;
        }

        diff_parm_triplet_offset_count.scan();

        auto M             = total_frame_dof_count;
        auto N             = total_parm_count;
        auto triplet_count = diff_parm_triplet_offset_count.total_count();

        local_triplet_pGpP.reshape(M, N);
        local_triplet_pGpP.resize_triplets(triplet_count);

        for(auto&& R : diff_parm_reporter_view)
        {
            DiffParmInfo info{this, R->m_index};
            R->assemble_diff_parm(info);
        }

        detail::build_coo_matrix(ctx(),  //
                                 total_coo_pGpP,
                                 total_triplet_pGpP,
                                 local_triplet_pGpP);
    }

    // 2) Assemble the H Triplets
    {
        auto diff_dof_reporter_view  = diff_dof_reporters.view();
        auto diff_dof_triplet_counts = diff_dof_triplet_offset_count.counts();

        for(auto&& [i, R] : enumerate(diff_dof_reporter_view))
        {
            DiffDofExtentInfo info{this, R->m_index};
            R->report_extent(info);

            diff_dof_triplet_counts[i] = info.m_triplet_count;
        }

        diff_dof_triplet_offset_count.scan();

        auto N             = total_frame_dof_count;
        auto triplet_count = diff_dof_triplet_offset_count.total_count();

        local_triplet_H.reshape(N, N);
        local_triplet_H.resize_triplets(triplet_count);

        for(auto&& R : diff_dof_reporter_view)
        {
            DiffDofInfo info{this, R->m_index};
            R->assemble_diff_dof(info);
        }

        detail::build_coo_matrix(ctx(),  //
                                 total_coo_H,
                                 total_triplet_H,
                                 local_triplet_H);
    }
}

void GlobalDiffSimManager::Impl::write_scene(WorldVisitor& world)
{
    auto& diff_sim = world.scene().diff_sim();

    detail::copy_to_host(total_coo_pGpP, host_coo_pGpP);
    detail::copy_to_host(total_coo_H, host_coo_H);

    diff_sim.pGpP(host_coo_pGpP.view());
    diff_sim.H(host_coo_H.view());
}

void GlobalDiffSimManager::init()
{
    m_impl.init(world());
}

void GlobalDiffSimManager::assemble()
{
    m_impl.assemble();
}

void GlobalDiffSimManager::update()
{
    m_impl.update();
}

void GlobalDiffSimManager::add_reporter(DiffDofReporter* subsystem)
{
    UIPC_ASSERT(subsystem != nullptr, "subsystem is nullptr");
    m_impl.diff_dof_reporters.register_subsystem(*subsystem);
}

void GlobalDiffSimManager::add_reporter(DiffParmReporter* subsystem)
{
    UIPC_ASSERT(subsystem != nullptr, "subsystem is nullptr");
    m_impl.diff_parm_reporters.register_subsystem(*subsystem);
}

muda::TripletMatrixView<Float, 1> GlobalDiffSimManager::DiffParmInfo::pGpP() const
{
    auto offset = m_impl->diff_parm_triplet_offset_count.offsets()[m_index];
    auto count  = m_impl->diff_parm_triplet_offset_count.counts()[m_index];
    return m_impl->local_triplet_pGpP.view().subview(offset, count);
}

muda::TripletMatrixView<Float, 1> GlobalDiffSimManager::DiffDofInfo::H() const
{
    auto offset = m_impl->diff_dof_triplet_offset_count.offsets()[m_index];
    auto count  = m_impl->diff_dof_triplet_offset_count.counts()[m_index];
    return m_impl->local_triplet_H.view().subview(offset, count);
}

SizeT GlobalDiffSimManager::BaseInfo::frame() const
{
    return m_impl->sim_engine->frame();
}

IndexT GlobalDiffSimManager::BaseInfo::dof_offset(SizeT frame) const
{
    return m_impl->dof_offsets[frame - 1];  // we record from the frame 1
}

IndexT GlobalDiffSimManager::BaseInfo::dof_count(SizeT frame) const
{
    return m_impl->dof_counts[frame - 1];  // we record from the frame 1
}

diff_sim::SparseCOOView GlobalDiffSimManager::SparseCOO::view() const
{
    return diff_sim::SparseCOOView{row_indices, col_indices, values, shape};
}

muda::CBufferView<Float> GlobalDiffSimManager::DiffParmUpdateInfo::parameters() const noexcept
{
    return m_impl->parameters.view();
}
}  // namespace uipc::backend::cuda