#include <contact_system/global_contact_manager.h>
#include <sim_engine.h>
#include <contact_system/contact_reporter.h>
#include <contact_system/contact_receiver.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<GlobalContactManager>
{
  public:
    static U<GlobalContactManager> create(SimEngine& engine)
    {
        return CreatorQuery::is_contact_enabled(engine) ?
                   make_unique<GlobalContactManager>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(GlobalContactManager);

void GlobalContactManager::do_build()
{
    m_impl.global_vertex_manager = find<GlobalVertexManager>();
    const auto& info             = world().scene().info();
    m_impl.related_d_hat         = info["contact"]["d_hat"].get<Float>();

    on_init_scene([this] { m_impl.init(); });
}

void GlobalContactManager::Impl::compute_d_hat()
{
    AABB vert_aabb = global_vertex_manager->vertex_bounding_box();
    d_hat          = related_d_hat;  // TODO: just hard code for now
    d_hat          = 0.02;
}

void GlobalContactManager::Impl::compute_adaptive_kappa()
{
    kappa = 1e8;  // TODO: just hard code for now
        // TODO: just hard code for now

    contact_tabular.fill(ContactCoeff{
        .kappa = kappa,
        .mu    = 0.0,
    });
}

void GlobalContactManager::Impl::init()
{
    // tabular
    contact_tabular.resize(muda::Extent2D{1, 1});

    // reporters
    contact_reporters.reserve(contact_reporter_buffer.size());
    std::ranges::move(contact_reporter_buffer, std::back_inserter(contact_reporters));

    reporter_gradient_offsets.resize(contact_reporters.size());
    reporter_gradient_counts.resize(contact_reporters.size());

    reporter_hessian_offsets.resize(contact_reporters.size());
    reporter_hessian_counts.resize(contact_reporters.size());

    // receivers
    contact_receivers.reserve(contact_receiver_buffer.size());
    std::ranges::move(contact_receiver_buffer, std::back_inserter(contact_receivers));

    //classify_infos.resize(contact_receivers.size());
    classified_contact_gradients.resize(contact_receivers.size());
    classified_contact_hessians.resize(contact_receivers.size());
}

void GlobalContactManager::Impl::compute_contact()
{
    _assemble();
    _convert_matrix();
    _distribute();
}

void GlobalContactManager::Impl::_assemble()
{
    auto vertex_count = global_vertex_manager->positions().size();

    for(auto&& [i, reporter] : enumerate(contact_reporters))
    {
        ContactExtentInfo info;
        reporter->report_extent(info);
        reporter_gradient_counts[i] = info.m_gradient_count;
        reporter_hessian_counts[i]  = info.m_hessian_count;
        spdlog::info("reporter {} gradient count: {}, hessian count: {}",
                     i,
                     info.m_gradient_count,
                     info.m_hessian_count);
    }

    // scan
    std::exclusive_scan(reporter_gradient_counts.begin(),
                        reporter_gradient_counts.end(),
                        reporter_gradient_offsets.begin(),
                        0);
    std::exclusive_scan(reporter_hessian_counts.begin(),
                        reporter_hessian_counts.end(),
                        reporter_hessian_offsets.begin(),
                        0);

    auto total_gradient_count =
        reporter_gradient_offsets.back() + reporter_gradient_counts.back();
    auto total_hessian_count =
        reporter_hessian_offsets.back() + reporter_hessian_counts.back();

    // allocate
    loose_resize_entries(collected_contact_gradient, total_gradient_count);
    loose_resize_entries(sorted_contact_gradient, total_gradient_count);
    loose_resize_entries(collected_contact_hessian, total_hessian_count);
    loose_resize_entries(sorted_contact_hessian, total_hessian_count);
    collected_contact_gradient.reshape(vertex_count);
    collected_contact_hessian.reshape(vertex_count, vertex_count);

    // collect
    for(auto&& [i, reporter] : enumerate(contact_reporters))
    {
        auto g_offset = reporter_gradient_offsets[i];
        auto g_count  = reporter_gradient_counts[i];
        auto h_offset = reporter_hessian_offsets[i];
        auto h_count  = reporter_hessian_counts[i];

        ContactInfo info;

        info.m_gradient = collected_contact_gradient.view().subview(g_offset, g_count);
        info.m_hessian = collected_contact_hessian.view().subview(h_offset, h_count);

        reporter->assemble(info);
    }
}

void GlobalContactManager::Impl::_convert_matrix()
{
    matrix_converter.convert(collected_contact_hessian, sorted_contact_hessian);
    matrix_converter.convert(collected_contact_gradient, sorted_contact_gradient);
}

void GlobalContactManager::Impl::_distribute()
{
    using namespace muda;

    auto vertex_count = global_vertex_manager->positions().size();

    for(auto&& [i, receiver] : enumerate(contact_receivers))
    {
        ClassifyInfo info;
        receiver->report(info);

        auto& classified_gradients = classified_contact_gradients[i];
        classified_gradients.reshape(vertex_count);
        auto& classified_hessians = classified_contact_hessians[i];
        classified_hessians.reshape(vertex_count, vertex_count);

        // 1) report gradient
        if(!info.is_empty())
        {
            const auto N = sorted_contact_gradient.doublet_count();

            // clear the range in device
            gradient_range = Vector2i{0, 0};

            // partition
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(N,
                       [gradient_range = gradient_range.viewer().name("gradient_range"),
                        contact_gradient =
                            std::as_const(sorted_contact_gradient).viewer().name("contact_gradient"),
                        range = info.m_gradient_i_range] __device__(int I) mutable
                       {
                           auto in_range = [](int i, const Vector2i& range)
                           { return i >= range.x() && i < range.y(); };

                           auto&& [i, G]      = contact_gradient(I);
                           bool this_in_range = in_range(i, range);

                           //cout << "I: " << I << ", i: " << i << ", G: " << G
                           //     << ", in_range: " << this_in_range << "\n";

                           if(!this_in_range)
                           {
                               return;
                           }

                           bool prev_in_range = false;
                           if(I > 0)
                           {
                               auto&& [prev_i, prev_G] = contact_gradient(I - 1);
                               prev_in_range = in_range(prev_i, range);
                           }
                           bool next_in_range = false;
                           if(I < contact_gradient.total_doublet_count() - 1)
                           {
                               auto&& [next_i, next_G] = contact_gradient(I + 1);
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
                    .kernel_name(__FUNCTION__ "-gradient")
                    .apply(count,
                           [contact_gradient =
                                std::as_const(sorted_contact_gradient).viewer().name("contact_gradient"),
                            classified_gradient = classified_gradients.viewer().name("classified_gradient"),
                            range = h_range] __device__(int I) mutable
                           {
                               auto&& [i, G] = contact_gradient(range.x() + I);
                               classified_gradient(I).write(i, G);
                           });
            }
        }

        // 2) report hessian
        if(!info.is_empty())
        {
            const auto N = sorted_contact_hessian.triplet_count();

            // +1 for calculate the total count
            loose_resize(selected_hessian, N + 1);
            loose_resize(selected_hessian_offsets, N + 1);

            // select
            ParallelFor()
                .kernel_name(__FUNCTION__ "-hessian")
                .apply(N,
                       [selected_hessian = selected_hessian.view(0, N).viewer().name("selected_hessian"),
                        last = VarView{selected_hessian.data() + N}.viewer().name("last"),
                        contact_hessian = sorted_contact_hessian.cviewer().name("contact_hessian"),
                        i_range = info.m_hessian_i_range,
                        j_range = info.m_hessian_j_range] __device__(int I) mutable
                       {
                           auto&& [i, j, H] = contact_hessian(I);

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
            VarView{selected_hessian_offsets.data() + N}.copy_to(&h_total_count);

            loose_resize_entries(classified_hessians, h_total_count);

            // fill
            if(h_total_count > 0)
            {
                ParallelFor()
                    .kernel_name(__FUNCTION__ "-hessian-fill")
                    .apply(N,
                           [selected_hessian = selected_hessian.cviewer().name("selected_hessian"),
                            selected_hessian_offsets =
                                selected_hessian_offsets.cviewer().name("selected_hessian_offsets"),
                            contact_hessian = sorted_contact_hessian.cviewer().name("contact_hessian"),
                            classified_hessian = classified_hessians.viewer().name("classified_hessian"),
                            i_range = info.m_hessian_i_range,
                            j_range = info.m_hessian_j_range] __device__(int I) mutable
                           {
                               if(selected_hessian(I))
                               {
                                   auto&& [i, j, H] = contact_hessian(I);
                                   auto offset = selected_hessian_offsets(I);

                                   classified_hessian(offset).write(i, j, H);
                               }
                           });
            }

            ClassifiedContactInfo classified_info;

            classified_info.m_gradient = classified_gradients.view();
            classified_info.m_hessian  = classified_hessians.view();

            receiver->receive(classified_info);
        }
    }
}

void GlobalContactManager::Impl::loose_resize_entries(muda::DeviceTripletMatrix<Float, 3>& m,
                                                      SizeT size)
{
    if(size > m.triplet_capacity())
    {
        m.reserve_triplets(size * reserve_ratio);
    }
    m.resize_triplets(size);
}

void GlobalContactManager::Impl::loose_resize_entries(muda::DeviceDoubletVector<Float, 3>& v,
                                                      SizeT size)
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
void GlobalContactManager::compute_d_hat()
{
    m_impl.compute_d_hat();
}

void GlobalContactManager::compute_contact()
{
    m_impl.compute_contact();
}

void GlobalContactManager::compute_adaptive_kappa()
{
    m_impl.compute_adaptive_kappa();
}

Float GlobalContactManager::d_hat() const
{
    return m_impl.d_hat;
}
void GlobalContactManager::add_reporter(ContactReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    reporter->m_index = m_impl.contact_reporter_buffer.size();
    m_impl.contact_reporter_buffer.push_back(reporter);
}
void GlobalContactManager::add_receiver(ContactReceiver* receiver)
{
    check_state(SimEngineState::BuildSystems, "add_receiver()");
    UIPC_ASSERT(receiver != nullptr, "receiver is nullptr");
    receiver->m_index = m_impl.contact_receiver_buffer.size();
    m_impl.contact_receiver_buffer.push_back(receiver);
}
muda::CBuffer2DView<ContactCoeff> GlobalContactManager::contact_tabular() const noexcept
{
    return m_impl.contact_tabular;
}
}  // namespace uipc::backend::cuda