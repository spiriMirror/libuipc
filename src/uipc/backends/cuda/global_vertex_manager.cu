#include <global_vertex_manager.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <muda/cub/device/device_reduce.h>

/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
void GlobalVertexManager::Impl::build_vertex_info()
{
    vertex_registers.reserve(vertex_registers_buffer.size());
    std::ranges::move(vertex_registers_buffer, std::back_inserter(vertex_registers));

    auto N = vertex_registers.size();
    register_vertex_counts.resize(N);
    register_vertex_offsets.resize(N);

    for(auto&& [i, R] : enumerate(vertex_registers))
    {
        VertexCountInfo info;
        R.m_report_vertex_count(info);
        // get count back
        register_vertex_counts[i] = info.count();
    }

    std::exclusive_scan(register_vertex_counts.begin(),
                        register_vertex_counts.end(),
                        register_vertex_offsets.begin(),
                        0);

    auto total_count = register_vertex_offsets.back() + register_vertex_counts.back();

    // resize the global coindex buffer
    coindex.resize(total_count);
    positions.resize(total_count);
    displacements.resize(total_count, Vector3::Zero());

    // create the subviews for each attribute_reporter
    for(auto&& [i, R] : enumerate(vertex_registers))
    {
        VertexAttributeInfo attributes{this, i};
        R.m_report_vertex_attributes(attributes);
    }
}

Float GlobalVertexManager::Impl::compute_max_displacement()
{
    collect_vertex_displacements();

    muda::DeviceReduce().Reduce((Float*)displacements.data(),
                                max_disp.data(),
                                displacements.size() * 3,
                                [] CUB_RUNTIME_FUNCTION(const Float& L, const Float& R)
                                {
                                    auto absL = std::abs(L);
                                    auto absR = std::abs(R);
                                    return absL > absR ? absL : absR;
                                },
                                0.0);
    return max_disp;
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
    return AABB{min_pos_host, max_pos_host};
}
GlobalVertexManager::VertexRegister::VertexRegister(
    std::string_view                            name,
    std::function<void(VertexCountInfo&)>&&     report_vertex_count,
    std::function<void(VertexAttributeInfo&)>&& report_vertex_attributes,
    std::function<void(VertexDisplacementInfo&)>&& report_vertex_displacement) noexcept
    : m_name(name)
    , m_report_vertex_count(std::move(report_vertex_count))
    , m_report_vertex_attributes(std::move(report_vertex_attributes))
    , m_report_vertex_displacement(std::move(report_vertex_displacement))
{
}
}  // namespace uipc::backend::cuda


/*************************************************************************************************
* API Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalVertexManager);

void GlobalVertexManager::VertexCountInfo::count(SizeT count) noexcept
{
    m_count = count;
}

SizeT GlobalVertexManager::VertexCountInfo::count() const noexcept
{
    return m_count;
}

GlobalVertexManager::VertexAttributeInfo::VertexAttributeInfo(Impl* impl, SizeT index) noexcept
    : m_impl(impl)
    , m_index(index)
{
}

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributeInfo::coindex() const noexcept
{
    return m_impl->subview(m_impl->coindex, m_index);
}

muda::BufferView<Vector3> GlobalVertexManager::VertexAttributeInfo::positions() const noexcept
{
    return m_impl->subview(m_impl->positions, m_index);
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

muda::CBufferView<IndexT> GlobalVertexManager::VertexDisplacementInfo::coindex() const noexcept
{
    return m_impl->subview(m_impl->coindex, m_index);
}

void GlobalVertexManager::build_vertex_info()
{
    m_impl.build_vertex_info();
}

void GlobalVertexManager::on_update(std::string_view name,
                                    std::function<void(VertexCountInfo&)>&& report_vertex_count,
                                    std::function<void(VertexAttributeInfo&)>&& report_vertex_attributes,
                                    std::function<void(VertexDisplacementInfo&)>&& report_vertex_displacement)
{
    check_state(SimEngineState::BuildSystems, "on_update()");
    m_impl.on_update(VertexRegister{name,
                                    std::move(report_vertex_count),
                                    std::move(report_vertex_attributes),
                                    std::move(report_vertex_displacement)});
}

void GlobalVertexManager::Impl::on_update(VertexRegister&& vertex_register)
{
    vertex_registers_buffer.push_back(std::move(vertex_register));
}

muda::CBufferView<IndexT> GlobalVertexManager::coindex() const noexcept
{
    return m_impl.coindex;
}

muda::CBufferView<Vector3> GlobalVertexManager::positions() const noexcept
{
    return m_impl.positions;
}

muda::CBufferView<Vector3> GlobalVertexManager::displacements() const noexcept
{
    return m_impl.displacements;
}

void GlobalVertexManager::Impl::collect_vertex_displacements()
{
    for(auto&& [i, R] : enumerate(vertex_registers))
    {
        VertexDisplacementInfo vd{this, i};
        R.m_report_vertex_displacement(vd);
    }
}

Float GlobalVertexManager::compute_max_displacement()
{
    return m_impl.compute_max_displacement();
}

AABB GlobalVertexManager::compute_vertex_bounding_box()
{
    return m_impl.compute_vertex_bounding_box();
}
}  // namespace uipc::backend::cuda