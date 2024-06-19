#include <global_vertex_manager.h>
#include <uipc/common/enumerate.h>
/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
void GlobalVertexManager::build_vertex_info()
{
    auto          N = m_reporters.size();
    vector<SizeT> counts(N);
    vector<SizeT> offsets(N);

    for(auto&& [i, reporter] : enumerate(m_reporters))
    {
        VertexCountInfo info;
        reporter(info);
        // get count back
        counts[i] = info.count();
    }

    std::exclusive_scan(counts.begin(), counts.end(), offsets.begin(), 0);

    auto total_count = offsets.back() + counts.back();

    // resize the global coindex buffer
    m_coindex.resize(total_count);

    // create the subviews for each receiver
    for(auto&& [i, receiver] : enumerate(m_receivers))
    {
        GlobalVertexInfo info(m_coindex.view(offsets[i], counts[i]));
        receiver(info);
    }
}
}  // namespace uipc::backend::cuda


/*************************************************************************************************
* API Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalVertexManager);

void VertexCountInfo::count(SizeT count) noexcept
{
    m_count = count;
}

SizeT VertexCountInfo::count() const noexcept
{
    return m_count;
}

GlobalVertexInfo::GlobalVertexInfo(muda::BufferView<IndexT> coindex) noexcept
    : m_coindex(coindex)
{
}

muda::BufferView<IndexT> GlobalVertexInfo::coindex() const noexcept
{
    return m_coindex;
}

SizeT GlobalVertexInfo::offset() const noexcept
{
    return m_coindex.offset();
}

SizeT GlobalVertexInfo::count() const noexcept
{
    return m_coindex.size();
}

void GlobalVertexManager::on_update(std::function<void(VertexCountInfo&)>&& reporter,
                                    std::function<void(const GlobalVertexInfo&)>&& receiver)
{
    check_state(SimEngineState::BuildSystems, "on_update()");
    m_reporters.push_back(std::move(reporter));
    m_receivers.push_back(std::move(receiver));
}
muda::CBufferView<IndexT> GlobalVertexManager::coindex() const noexcept
{
    return m_coindex;
}
}  // namespace uipc::backend::cuda