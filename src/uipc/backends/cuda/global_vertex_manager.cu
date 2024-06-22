#include <global_vertex_manager.h>
#include <uipc/common/enumerate.h>
#include <muda/cub/device/device_reduce.h>

/*************************************************************************************************
* Core Implementation
*************************************************************************************************/
namespace uipc::backend::cuda
{
void GlobalVertexManager::Impl::build_vertex_info()
{
    auto          N = vertex_registers.size();
    vector<SizeT> counts(N);
    vector<SizeT> offsets(N);

    for(auto&& [i, R] : enumerate(vertex_registers))
    {
        VertexCountInfo info;
        R.m_report_vertex_count(info);
        // get count back
        counts[i] = info.count();
    }

    std::exclusive_scan(counts.begin(), counts.end(), offsets.begin(), 0);

    auto total_count = offsets.back() + counts.back();

    // resize the global coindex buffer
    coindex.resize(total_count);
    positions.resize(total_count);
    displacements.resize(total_count, Vector3::Zero());

    // create the subviews for each attribute_reporter
    for(auto&& [i, R] : enumerate(vertex_registers))
    {
        VertexAttributes attributes;
        auto             offset = offsets[i];
        auto             count  = counts[i];

        attributes.m_coindex   = coindex.view(offset, count);
        attributes.m_positions = positions.view(offset, count);
        R.m_report_vertex_attributes(attributes);
    }
}

Float GlobalVertexManager::Impl::compute_max_displacement()
{
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
    std::string_view                         name,
    std::function<void(VertexCountInfo&)>&&  report_vertex_count,
    std::function<void(VertexAttributes&)>&& report_vertex_attributes,
    std::function<void(VertexDisplacement&)>&& report_vertex_displacement) noexcept
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

muda::BufferView<IndexT> GlobalVertexManager::VertexAttributes::coindex() const noexcept
{
    return m_coindex;
}

muda::BufferView<Vector3> GlobalVertexManager::VertexAttributes::positions() const noexcept
{
    return m_positions;
}

muda::BufferView<Vector3> GlobalVertexManager::VertexDisplacement::displacements() const noexcept
{
    return m_displacements;
}

void GlobalVertexManager::build_vertex_info()
{
    m_impl.build_vertex_info();
}

void GlobalVertexManager::on_update(std::string_view name,
                                    std::function<void(VertexCountInfo&)>&& report_vertex_count,
                                    std::function<void(VertexAttributes&)>&& report_vertex_attributes,
                                    std::function<void(VertexDisplacement&)>&& report_vertex_displacement)
{
    check_state(SimEngineState::BuildSystems, "on_update()");
    m_impl.on_update(VertexRegister{name,
                                    std::move(report_vertex_count),
                                    std::move(report_vertex_attributes),
                                    std::move(report_vertex_displacement)});
}

void GlobalVertexManager::Impl::on_update(VertexRegister&& vertex_register)
{
    vertex_registers.push_back(std::move(vertex_register));
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

Float GlobalVertexManager::compute_max_displacement()
{
    return m_impl.compute_max_displacement();
}

AABB GlobalVertexManager::compute_vertex_bounding_box()
{
    return m_impl.compute_vertex_bounding_box();
}
}  // namespace uipc::backend::cuda