#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>
#include <functional>

namespace uipc::backend::cuda
{
class VertexCountInfo
{
  public:
    void  count(SizeT count) noexcept;
    SizeT count() const noexcept;

  private:
    SizeT m_count;
};

class GlobalVertexInfo
{
  public:
    GlobalVertexInfo() = default;
    GlobalVertexInfo(muda::BufferView<IndexT> coindex) noexcept;
    muda::BufferView<IndexT> coindex() const noexcept;
    SizeT                    offset() const noexcept;
    SizeT                    count() const noexcept;

  private:
    muda::BufferView<IndexT> m_coindex;
};

class GlobalVertexManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    void on_update(std::function<void(VertexCountInfo&)>&&        reporter,
                   std::function<void(const GlobalVertexInfo&)>&& receiver);

    muda::CBufferView<IndexT> coindex() const noexcept;

  private:
    /**
     * @brief A mapping from the global vertex index to the coindex.
     * 
     * The values of coindex is dependent on the reporters, which can be:
     * 1) the local index of the vertex
     * 2) or any other information that is needed to be stored.
     */
    muda::DeviceBuffer<IndexT> m_coindex;

    list<std::function<void(VertexCountInfo&)>>        m_reporters;
    list<std::function<void(const GlobalVertexInfo&)>> m_receivers;

    friend class SimEngine;
    void build_vertex_info();  // only be called by SimEngine
};
}  // namespace uipc::backend::cuda