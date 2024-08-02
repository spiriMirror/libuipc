#include <contact_system/vertex_half_plane_normal_contact.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
void VertexHalfPlaneNormalContact::do_build()
{
    m_impl.global_trajectory_filter = &require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = &require<GlobalContactManager>();
    m_impl.global_vertex_manager    = &require<GlobalVertexManager>();

    BuildInfo info;
    do_build(info);

    m_impl.global_contact_manager->add_reporter(this);
    m_impl.dt = world().scene().info()["dt"].get<Float>();

    on_init_scene(
        [this]
        {
            m_impl.veretx_half_plane_trajectory_filter =
                m_impl.global_trajectory_filter->find<VertexHalfPlaneTrajectoryFilter>();
        });
}

void VertexHalfPlaneNormalContact::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    SizeT count = filter->PHs().size();

    info.gradient_count(count);
    info.hessian_count(count);

    m_impl.loose_resize(m_impl.gradients, count);
    m_impl.loose_resize(m_impl.hessians, count);
}

void VertexHalfPlaneNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;

    EnergyInfo this_info{&m_impl};

    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    auto count = filter->PHs().size();

    m_impl.loose_resize(m_impl.energies, count);
    this_info.m_energies = m_impl.energies.view();

    // let subclass to fill in the data
    do_compute_energy(this_info);

    DeviceReduce().Sum(
        m_impl.energies.data(), info.energy().data(), m_impl.energies.size());

    Float E;
    info.energy().copy_to(&E);

    spdlog::info("VertexHalfPlaneNormalContact energy: {}", E);
}

namespace detail
{
    template <SizeT N>
    __inline__ __device__ void make_spd(Matrix<Float, N, N>& mat)
    {
        Vector<Float, N>    eigen_values;
        Matrix<Float, N, N> eigen_vectors;
        muda::eigen::template evd(mat, eigen_values, eigen_vectors);
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto& v = eigen_values(i);
            v       = v < 0.0 ? 0.0 : v;
        }
        mat = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    }
}  // namespace detail


void VertexHalfPlaneNormalContact::Impl::assemble(GlobalContactManager::ContactInfo& info)
{
    using namespace muda;

    auto H3x3 = info.hessian();
    auto G3   = info.gradient();
    auto PHs  = veretx_half_plane_trajectory_filter->PHs();

    // PH
    {
        // Hessians
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(hessians.size(),
                   [PH_H3x3s = hessians.cviewer().name("PH_H3x3"),
                    PHs      = PHs.cviewer().name("PHs"),
                    H3x3s = H3x3.viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix3x3   H3x3;
                       const auto& D2 = PHs(I);
                       auto        P  = D2.x();
                       //detail::make_spd<3>(H3x3);
                       H3x3s(I).write(P, P, H3x3);
                   });

        // Gradients
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(gradients.size(),
                   [PH_G3s = gradients.cviewer().name("PHG3"),
                    PHs    = PHs.cviewer().name("PHs"),
                    G3s    = G3.viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& PG3 = PH_G3s(I);
                       const auto& D2  = PHs(I);
                       auto        P   = D2.x();
                       G3s(I).write(P, PG3);
                   });
    }
}

void VertexHalfPlaneNormalContact::do_assemble(GlobalContactManager::ContactInfo& info)
{
    ContactInfo this_info{&m_impl};

    this_info.m_gradients = m_impl.gradients;
    this_info.m_hessians  = m_impl.hessians;

    // let subclass to fill in the data
    do_assemble(this_info);

    // assemble the data to the global contact manager
    m_impl.assemble(info);
}

muda::CBuffer2DView<ContactCoeff> VertexHalfPlaneNormalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector2i> VertexHalfPlaneNormalContact::BaseInfo::PHs() const
{
    return m_impl->veretx_half_plane_trajectory_filter->PHs();
}

muda::CBufferView<Vector3> VertexHalfPlaneNormalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> VertexHalfPlaneNormalContact::BaseInfo::prev_positions() const
{
    return m_impl->global_vertex_manager->prev_positions();
}

muda::CBufferView<Vector3> VertexHalfPlaneNormalContact::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

muda::CBufferView<IndexT> VertexHalfPlaneNormalContact::BaseInfo::contact_element_ids() const
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

Float VertexHalfPlaneNormalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float VertexHalfPlaneNormalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float VertexHalfPlaneNormalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}

muda::BufferView<Vector3> VertexHalfPlaneNormalContact::ContactInfo::gradients() const noexcept
{
    return m_gradients;
}
muda::BufferView<Matrix3x3> VertexHalfPlaneNormalContact::ContactInfo::hessians() const noexcept
{
    return m_hessians;
}
muda::BufferView<Float> VertexHalfPlaneNormalContact::EnergyInfo::energies() const noexcept
{
    return m_energies;
}
}  // namespace uipc::backend::cuda
