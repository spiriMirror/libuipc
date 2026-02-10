#include <sim_engine.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <dytopo_effect_system/dytopo_effect_reporter.h>
#include <dytopo_effect_system/dytopo_effect_receiver.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>
#include <uipc/common/unit.h>
#include <uipc/common/zip.h>
#include <energy_component_flags.h>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalDyTopoEffectManager>
{
  public:
    static U<cuda::GlobalDyTopoEffectManager> create(cuda::SimEngine& engine)
    {
        auto dytopo_effect_enable_attr =
            engine.world().scene().config().find<IndexT>("contact/enable");
        bool dytopo_effect_enable = dytopo_effect_enable_attr->view()[0] != 0;

        auto& types = engine.world().scene().constitution_tabular().types();
        bool  has_inter_primitive_constitution =
            types.find(std::string{builtin::InterPrimitive}) != types.end();

        if(dytopo_effect_enable || has_inter_primitive_constitution)
            return make_unique<cuda::GlobalDyTopoEffectManager>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalDyTopoEffectManager);

muda::CBCOOVectorView<Float, 3> GlobalDyTopoEffectManager::gradients() const noexcept
{
    return m_impl.sorted_dytopo_effect_gradient.view();
}

muda::CBCOOMatrixView<Float, 3> GlobalDyTopoEffectManager::hessians() const noexcept
{
    return m_impl.sorted_dytopo_effect_hessian.view();
}

void GlobalDyTopoEffectManager::do_build()
{
    const auto& config = world().scene().config();

    m_impl.global_vertex_manager = require<GlobalVertexManager>();
}

void GlobalDyTopoEffectManager::Impl::init(WorldVisitor& world)
{
    // 3) reporters
    auto dytopo_effect_reporter_view = dytopo_effect_reporters.view();
    for(auto&& [i, R] : enumerate(dytopo_effect_reporter_view))
        R->init();
    for(auto&& [i, R] : enumerate(dytopo_effect_reporter_view))
        R->m_index = i;

    reporter_energy_offsets_counts.resize(dytopo_effect_reporter_view.size());
    reporter_gradient_offsets_counts.resize(dytopo_effect_reporter_view.size());
    reporter_hessian_offsets_counts.resize(dytopo_effect_reporter_view.size());

    // 4) receivers
    auto dytopo_effect_receiver_view = dytopo_effect_receivers.view();
    for(auto&& [i, R] : enumerate(dytopo_effect_receiver_view))
        R->init();
    for(auto&& [i, R] : enumerate(dytopo_effect_receiver_view))
        R->m_index = i;

    classified_dytopo_effect_gradients.resize(dytopo_effect_receiver_view.size());
    classified_dytopo_effect_hessians.resize(dytopo_effect_receiver_view.size());
}

void GlobalDyTopoEffectManager::Impl::compute_dytopo_effect(ComputeDyTopoEffectInfo& info)
{
    _assemble(info);
    _convert_matrix();
    _distribute(info);
}

void GlobalDyTopoEffectManager::Impl::_assemble(ComputeDyTopoEffectInfo& info)
{
    Timer timer{"Assemble Dytopo Effect"};

    auto vertex_count = global_vertex_manager->positions().size();

    auto reporter_gradient_counts = reporter_gradient_offsets_counts.counts();
    auto reporter_hessian_counts  = reporter_hessian_offsets_counts.counts();
    bool gradient_only            = info.m_gradient_only;

    logger::info("DyTopo Effect Assembly: GradientOnly={}, ComponentFlags={}",
                 info.m_gradient_only,
                 enum_flags_name(info.m_component_flags));

    for(auto&& [i, reporter] : enumerate(dytopo_effect_reporters.view()))
    {
        reporter_gradient_counts[i] = 0;
        reporter_hessian_counts[i]  = 0;

        if(!has_flags(info.m_component_flags, reporter->component_flags()))
            continue;

        GradientHessianExtentInfo extent_info;
        extent_info.m_gradient_only = gradient_only;
        reporter->report_gradient_hessian_extent(extent_info);

        reporter_gradient_counts[i] = extent_info.m_gradient_count;
        reporter_hessian_counts[i] = gradient_only ? 0 : extent_info.m_hessian_count;
        logger::info("<{}> DyTopo Grad3 count: {}, DyTopo Hess3x3 count: {}",
                     reporter->name(),
                     extent_info.m_gradient_count,
                     extent_info.m_hessian_count);
    }

    // scan
    reporter_gradient_offsets_counts.scan();
    reporter_hessian_offsets_counts.scan();

    auto total_gradient_count = reporter_gradient_offsets_counts.total_count();
    auto total_hessian_count  = reporter_hessian_offsets_counts.total_count();

    // allocate
    loose_resize_entries(collected_dytopo_effect_gradient, total_gradient_count);
    loose_resize_entries(sorted_dytopo_effect_gradient, total_gradient_count);
    loose_resize_entries(collected_dytopo_effect_hessian, total_hessian_count);
    loose_resize_entries(sorted_dytopo_effect_hessian, total_hessian_count);
    collected_dytopo_effect_gradient.reshape(vertex_count);
    collected_dytopo_effect_hessian.reshape(vertex_count, vertex_count);

    // collect
    for(auto&& [i, reporter] : enumerate(dytopo_effect_reporters.view()))
    {
        if(!has_flags(info.m_component_flags, reporter->component_flags()))
            continue;

        auto [g_offset, g_count] = reporter_gradient_offsets_counts[i];
        auto [h_offset, h_count] = reporter_hessian_offsets_counts[i];

        GradientHessianInfo info;
        info.m_gradient_only = gradient_only;

        info.m_gradients =
            collected_dytopo_effect_gradient.view().subview(g_offset, g_count);
        info.m_hessians = collected_dytopo_effect_hessian.view().subview(h_offset, h_count);

        reporter->assemble(info);
    }
}

void GlobalDyTopoEffectManager::Impl::_convert_matrix()
{
    Timer timer{"Convert Dytopo Matrix"};

    matrix_converter.convert(collected_dytopo_effect_hessian, sorted_dytopo_effect_hessian);
    matrix_converter.convert(collected_dytopo_effect_gradient, sorted_dytopo_effect_gradient);
}

void GlobalDyTopoEffectManager::Impl::_distribute(ComputeDyTopoEffectInfo& info)
{
    Timer timer{"Distribute Dytopo Effect"};

    using namespace muda;

    auto vertex_count = global_vertex_manager->positions().size();

    for(auto&& [i, receiver] : enumerate(dytopo_effect_receivers.view()))
    {
        DyTopoClassifyInfo classify_info;
        receiver->report(classify_info);


        ClassifiedDyTopoEffectInfo classified_info;
        auto& classified_gradients = classified_dytopo_effect_gradients[i];
        classified_gradients.reshape(vertex_count);
        auto& classified_hessians = classified_dytopo_effect_hessians[i];
        classified_hessians.reshape(vertex_count, vertex_count);

        // 1) report gradient
        if(classify_info.is_diag())
        {
            const auto N = sorted_dytopo_effect_gradient.doublet_count();

            // clear the range in device
            gradient_range = Vector2i{0, 0};

            // partition
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(
                    N,
                    [gradient_range = gradient_range.viewer().name("gradient_range"),
                     dytopo_effect_gradient =
                         std::as_const(sorted_dytopo_effect_gradient).viewer().name("dytopo_effect_gradient"),
                     range = classify_info.gradient_i_range()] __device__(int I) mutable
                    {
                        auto in_range = [](int i, const Vector2i& range)
                        { return i >= range.x() && i < range.y(); };

                        auto&& [i, G]      = dytopo_effect_gradient(I);
                        bool this_in_range = in_range(i, range);

                        if(!this_in_range)
                        {
                            return;
                        }

                        bool prev_in_range = false;
                        if(I > 0)
                        {
                            auto&& [prev_i, prev_G] = dytopo_effect_gradient(I - 1);
                            prev_in_range = in_range(prev_i, range);
                        }
                        bool next_in_range = false;
                        if(I < dytopo_effect_gradient.total_doublet_count() - 1)
                        {
                            auto&& [next_i, next_G] = dytopo_effect_gradient(I + 1);
                            next_in_range = in_range(next_i, range);
                        }

                        // if the prev is not in range, then this is the start of the partition
                        if(!prev_in_range)
                        {
                            gradient_range->x() = I;
                        }
                        // if the next is not in range, then this is the end of the partition
                        if(!next_in_range)
                        {
                            gradient_range->y() = I + 1;
                        }
                    });

            Vector2i h_range = gradient_range;  // copy back

            auto count = h_range.y() - h_range.x();

            loose_resize_entries(classified_gradients, count);

            // fill
            if(count > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(count,
                           [dytopo_effect_gradient = std::as_const(sorted_dytopo_effect_gradient)
                                                         .viewer()
                                                         .name("dytopo_effect_gradient"),
                            classified_gradient = classified_gradients.viewer().name("classified_gradient"),
                            range = h_range] __device__(int I) mutable
                           {
                               auto&& [i, G] = dytopo_effect_gradient(range.x() + I);
                               classified_gradient(I).write(i, G);
                           });
            }

            classified_info.m_gradients = classified_gradients.view();
        }

        // 2) report hessian
        if(!info.m_gradient_only && !classify_info.is_empty())
        {
            const auto N = sorted_dytopo_effect_hessian.triplet_count();

            // +1 for calculate the total count
            loose_resize(selected_hessian, N + 1);
            loose_resize(selected_hessian_offsets, N + 1);

            // select
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(
                    N,
                    [selected_hessian = selected_hessian.view(0, N).viewer().name("selected_hessian"),
                     last =
                         VarView<IndexT>{selected_hessian.data() + N}.viewer().name("last"),
                     dytopo_effect_hessian =
                         sorted_dytopo_effect_hessian.cviewer().name("dytopo_effect_hessian"),
                     i_range = classify_info.hessian_i_range(),
                     j_range = classify_info.hessian_j_range()] __device__(int I) mutable
                    {
                        auto&& [i, j, H] = dytopo_effect_hessian(I);

                        auto in_range = [](int i, const Vector2i& range)
                        { return i >= range.x() && i < range.y(); };

                        selected_hessian(I) =
                            in_range(i, i_range) && in_range(j, j_range) ? 1 : 0;

                        // fill the last one as 0, so that we can calculate the total count
                        // during the exclusive scan
                        if(I == 0)
                            last = 0;
                    });

            // scan
            DeviceScan().ExclusiveSum(selected_hessian.data(),
                                      selected_hessian_offsets.data(),
                                      selected_hessian.size());

            IndexT h_total_count = 0;
            VarView<IndexT>{selected_hessian_offsets.data() + N}.copy_to(&h_total_count);

            loose_resize_entries(classified_hessians, h_total_count);

            // fill
            if(h_total_count > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(N,
                           [selected_hessian = selected_hessian.cviewer().name("selected_hessian"),
                            selected_hessian_offsets =
                                selected_hessian_offsets.cviewer().name("selected_hessian_offsets"),
                            dytopo_effect_hessian =
                                sorted_dytopo_effect_hessian.cviewer().name("dytopo_effect_hessian"),
                            classified_hessian = classified_hessians.viewer().name("classified_hessian"),
                            i_range = classify_info.hessian_i_range(),
                            j_range = classify_info.hessian_j_range()] __device__(int I) mutable
                           {
                               if(selected_hessian(I))
                               {
                                   auto&& [i, j, H] = dytopo_effect_hessian(I);
                                   auto offset = selected_hessian_offsets(I);

                                   classified_hessian(offset).write(i, j, H);
                               }
                           });
            }

            classified_info.m_hessians = classified_hessians.view();
        }

        receiver->receive(classified_info);
    }
}

void GlobalDyTopoEffectManager::Impl::loose_resize_entries(
    muda::DeviceTripletMatrix<Float, 3>& m, SizeT size)
{
    if(size > m.triplet_capacity())
    {
        m.reserve_triplets(size * reserve_ratio);
    }
    m.resize_triplets(size);
}

void GlobalDyTopoEffectManager::Impl::loose_resize_entries(
    muda::DeviceDoubletVector<Float, 3>& v, SizeT size)
{
    if(size > v.doublet_capacity())
    {
        v.reserve_doublets(size * reserve_ratio);
    }
    v.resize_doublets(size);
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void GlobalDyTopoEffectManager::init()
{
    m_impl.init(world());
}

void GlobalDyTopoEffectManager::compute_dytopo_effect(ComputeDyTopoEffectInfo& info)
{
    m_impl.compute_dytopo_effect(info);
}

void GlobalDyTopoEffectManager::compute_dytopo_effect()
{
    ComputeDyTopoEffectInfo info;
    m_impl.compute_dytopo_effect(info);
}

void GlobalDyTopoEffectManager::add_reporter(DyTopoEffectReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    auto flag = reporter->component_flags();
    UIPC_ASSERT(is_valid_flag(flag),
                "reporter component_flags() is not valid single flag, it's {}",
                enum_flags_name(flag));
    m_impl.dytopo_effect_reporters.register_sim_system(*reporter);

    // classify into contact / non-contact
    if(reporter->component_flags() == EnergyComponentFlags::Contact)
    {
        m_impl.contact_reporters.register_sim_system(*reporter);
    }
    else
    {
        m_impl.non_contact_reporters.register_sim_system(*reporter);
    }
}

void GlobalDyTopoEffectManager::add_receiver(DyTopoEffectReceiver* receiver)
{
    check_state(SimEngineState::BuildSystems, "add_receiver()");
    UIPC_ASSERT(receiver != nullptr, "receiver is nullptr");
    m_impl.dytopo_effect_receivers.register_sim_system(*receiver);
}
}  // namespace uipc::backend::cuda