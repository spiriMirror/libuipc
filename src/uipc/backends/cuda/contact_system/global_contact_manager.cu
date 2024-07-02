#include <contact_system/global_contact_manager.h>
#include <sim_engine.h>
#include <contact_system/contact_reporter.h>
#include <uipc/common/enumerate.h>

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
}

void GlobalContactManager::Impl::init()
{
    contact_reporters.reserve(contact_reporter_buffer.size());
    std::ranges::move(contact_reporter_buffer, std::back_inserter(contact_reporters));

    contact_receivers.reserve(contact_receiver_buffer.size());
    std::ranges::move(contact_receiver_buffer, std::back_inserter(contact_receivers));

    reporter_gradient_offsets.resize(contact_reporters.size());
    reporter_gradient_counts.resize(contact_reporters.size());

    reporter_hessian_offsets.resize(contact_reporters.size());
    reporter_hessian_counts.resize(contact_reporters.size());
}

void GlobalContactManager::Impl::assemble()
{
    auto vertex_count = global_vertex_manager->positions().size();

    for(auto&& [i, reporter] : enumerate(contact_reporters))
    {
        ContactExtentInfo info;
        reporter->report_extent(info);
        reporter_gradient_counts[i] = info.m_gradient_count;
        reporter_hessian_counts[i]  = info.m_hessian_count;
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
    collected_contact_gradient.resize(vertex_count, total_gradient_count);
    collected_contact_hessian.resize(vertex_count, vertex_count, total_hessian_count);

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

void GlobalContactManager::Impl::compute_d_hat()
{
    AABB vert_aabb = global_vertex_manager->vertex_bounding_box();
    d_hat          = related_d_hat;  // TODO: just hard code for now
    d_hat          = 0.02;
}

void GlobalContactManager::Impl::compute_adaptive_kappa()
{
    kappa = 1e8;  // TODO: just hard code for now
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
    // collect contact count
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
    m_impl.contact_receiver_buffer.push_back(receiver);
}
}  // namespace uipc::backend::cuda