#include <global_geometry/global_body_manager.h>
#include <uipc/common/enumerate.h>
#include <global_geometry/body_reporter.h>

/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalBodyManager);

void GlobalBodyManager::do_build() {}

void GlobalBodyManager::Impl::init()
{
    auto body_reporter_vew = body_reporters.view();

    // 1) Setup index for each body reporter
    for(auto&& [i, R] : enumerate(body_reporter_vew))
        R->m_index = i;

    // 2) Call init on each body reporter
    for(auto&& R : body_reporter_vew)
        R->init();

    // 3) Count the number of bodies and prepare the offsets/counts for each body reporter
    auto N = body_reporter_vew.size();
    reporter_body_offsets_counts.resize(N);

    span<IndexT> reporter_body_counts = reporter_body_offsets_counts.counts();

    for(auto&& [i, R] : enumerate(body_reporter_vew))
    {
        BodyCountInfo info;
        R->report_count(info);
        // get count back
        reporter_body_counts[i] = info.m_count;
    }

    reporter_body_offsets_counts.scan();
    SizeT total_count = reporter_body_offsets_counts.total_count();

    // 4) Resize the attribute buffers
    coindices.resize(total_count);
    self_collision.resize(total_count);

    // 5) Let each body reporter fill the attributes
    for(auto&& [i, R] : enumerate(body_reporter_vew))
    {
        BodyAttributeInfo info{this, i};
        R->report_attributes(info);
    }
}
}  // namespace uipc::backend::cuda

/*************************************************************************************************
* API Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
void GlobalBodyManager::Impl::rebuild()
{
    UIPC_ASSERT(false, "Not implemented yet");
}

void GlobalBodyManager::init()
{
    m_impl.init();
}

void GlobalBodyManager::add_reporter(BodyReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter must not be null");
    m_impl.body_reporters.register_sim_system(*reporter);
}

muda::CBufferView<IndexT> GlobalBodyManager::coindices() const noexcept
{
    return m_impl.coindices;
}

muda::CBufferView<IndexT> GlobalBodyManager::self_collision() const noexcept
{
    return m_impl.self_collision;
}

muda::BufferView<IndexT> GlobalBodyManager::BodyAttributeInfo::coindices() const noexcept
{
    return m_impl->subview(m_impl->coindices, m_index);
}

muda::BufferView<IndexT> GlobalBodyManager::BodyAttributeInfo::self_collision() const noexcept
{
    return m_impl->subview(m_impl->self_collision, m_index);
}

GlobalBodyManager::BodyAttributeInfo::BodyAttributeInfo(Impl* impl, SizeT index) noexcept
    : m_impl(impl)
    , m_index(index)
{
}

void GlobalBodyManager::BodyCountInfo::count(SizeT count) noexcept
{
    m_count = count;
}

void GlobalBodyManager::BodyCountInfo::changeable(bool is_changable) noexcept
{
    m_changable = is_changable;
}
}  // namespace uipc::backend::cuda