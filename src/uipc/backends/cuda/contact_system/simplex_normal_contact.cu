#include <contact_system/simplex_normal_contact.h>
#include <muda/ext/eigen/evd.h>
#include <muda/cub/device/device_merge_sort.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>

namespace uipc::backend::cuda
{
void SimplexNormalContact::do_build()
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
            m_impl.simplex_trajectory_filter =
                m_impl.global_trajectory_filter->find<SimplexTrajectoryFilter>();
        });
}

void SimplexNormalContact::Impl::compute_energy(SimplexNormalContact* contact,
                                                GlobalContactManager::EnergyInfo& info)
{
    EnergyInfo this_info{this};

    auto filter = simplex_trajectory_filter;

    PT_count = filter->PTs().size();
    EE_count = filter->EEs().size();
    PE_count = filter->PEs().size();
    PP_count = filter->PPs().size();

    auto count_4 = (PT_count + EE_count);
    auto count_3 = PE_count;
    auto count_2 = PP_count;

    energies.resize(count_4 + count_3 + count_2);

    SizeT offset            = 0;
    this_info.m_PT_energies = energies.view(offset, PT_count);
    offset += PT_count;
    this_info.m_EE_energies = energies.view(offset, EE_count);
    offset += EE_count;
    this_info.m_PE_energies = energies.view(offset, PE_count);
    offset += PE_count;
    this_info.m_PP_energies = energies.view(offset, PP_count);


    contact->do_compute_energy(this_info);
    using namespace muda;

    // if(info.is_initial())
    //{
    //    DeviceMergeSort().SortKeys(energies.data(),
    //                               energies.size(),
    //                               [] CUB_RUNTIME_FUNCTION(Float a, Float b)
    //                               { return a < b; });
    //}

    DeviceReduce().Sum(energies.data(), info.energy().data(), energies.size());
}

void SimplexNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    m_impl.compute_energy(this, info);
}

void SimplexNormalContact::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;

    // m_impl.classify_constraints();
    //m_impl.PT_count = m_impl.PT_constraints.size();
    //m_impl.EE_count = m_impl.EE_constraints.size();
    //m_impl.PE_count = m_impl.PE_constraints.size();
    //m_impl.PP_count = m_impl.PP_constraints.size();

    m_impl.PT_count = filter->PTs().size();
    m_impl.EE_count = filter->EEs().size();
    m_impl.PE_count = filter->PEs().size();
    m_impl.PP_count = filter->PPs().size();


    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;

    // expand to hessian3x3 and graident3
    SizeT contact_gradient_count = 4 * count_4 + 3 * count_3 + 2 * count_2;
    SizeT contact_hessian_count = 4 * 4 * count_4 + 3 * 3 * count_3 + 2 * 2 * count_2;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(contact_hessian_count);

    m_impl.loose_resize(m_impl.PT_EE_gradients, count_4);
    m_impl.loose_resize(m_impl.PT_EE_hessians, count_4);

    m_impl.loose_resize(m_impl.PE_gradients, count_3);
    m_impl.loose_resize(m_impl.PE_hessians, count_3);

    m_impl.loose_resize(m_impl.PP_gradients, count_2);
    m_impl.loose_resize(m_impl.PP_hessians, count_2);
}

void SimplexNormalContact::do_assemble(GlobalContactManager::ContactInfo& info)
{
    ContactInfo this_info{&m_impl};

    this_info.m_PT_gradients = m_impl.PT_EE_gradients.view(0, m_impl.PT_count);
    this_info.m_EE_gradients =
        m_impl.PT_EE_gradients.view(m_impl.PT_count, m_impl.EE_count);
    this_info.m_PE_gradients = m_impl.PE_gradients.view();
    this_info.m_PP_gradients = m_impl.PP_gradients.view();

    this_info.m_PT_hessians = m_impl.PT_EE_hessians.view(0, m_impl.PT_count);
    this_info.m_EE_hessians = m_impl.PT_EE_hessians.view(m_impl.PT_count, m_impl.EE_count);
    this_info.m_PE_hessians = m_impl.PE_hessians.view();
    this_info.m_PP_hessians = m_impl.PP_hessians.view();

    // let subclass to fill in the data
    do_assemble(this_info);

    // _assemble the data to the global contact manager
    m_impl.assemble(info);
}

muda::CBuffer2DView<ContactCoeff> SimplexNormalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector4i> SimplexNormalContact::BaseInfo::PTs() const
{
    // return m_impl->PT_constraints.view();
    return m_impl->simplex_trajectory_filter->PTs();
}

muda::CBufferView<Vector4i> SimplexNormalContact::BaseInfo::EEs() const
{
    return m_impl->simplex_trajectory_filter->EEs();
}

muda::CBufferView<Vector3i> SimplexNormalContact::BaseInfo::PEs() const
{
    return m_impl->simplex_trajectory_filter->PEs();
}

muda::CBufferView<Vector2i> SimplexNormalContact::BaseInfo::PPs() const
{
    return m_impl->simplex_trajectory_filter->PPs();
}

muda::CBufferView<Float> SimplexNormalContact::BaseInfo::thicknesses() const
{
    return m_impl->global_vertex_manager->thicknesses();
}

muda::CBufferView<Vector3> SimplexNormalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> SimplexNormalContact::BaseInfo::prev_positions() const
{
    return m_impl->global_vertex_manager->prev_positions();
}

muda::CBufferView<Vector3> SimplexNormalContact::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

muda::CBufferView<IndexT> SimplexNormalContact::BaseInfo::contact_element_ids() const
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

Float SimplexNormalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float SimplexNormalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float SimplexNormalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}

namespace detail
{
    template <SizeT N>
    __inline__ __device__ void fill_contact_hessian(muda::TripletMatrixViewer<Float, 3>& H3x3,
                                                    int                      I,
                                                    const Vector<IndexT, N>& D,
                                                    const Matrix<Float, 3 * N, 3 * N>& H)
    {
        auto offset = N * N * I;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto L = D(i);
#pragma unroll
            for(int j = 0; j < N; ++j)
            {
                auto R = D(j);
                H3x3(offset++).write(L, R, H.template block<3, 3>(3 * i, 3 * j));
            }
        }
    }

    template <SizeT N>
    __inline__ __device__ void fill_contact_gradient(muda::DoubletVectorViewer<Float, 3>& G3,
                                                     int                      I,
                                                     const Vector<IndexT, N>& D,
                                                     const Vector<Float, 3 * N>& G)
    {
        auto offset = N * I;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto L = D(i);
            G3(offset++).write(L, G.segment<3>(3 * i));
        }
    }

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


void SimplexNormalContact::Impl::assemble(GlobalContactManager::ContactInfo& info)
{
    using namespace muda;

    auto H3x3 = info.hessian();
    auto G3   = info.gradient();
    auto PTs  = simplex_trajectory_filter->PTs();
    auto EEs  = simplex_trajectory_filter->EEs();
    auto PEs  = simplex_trajectory_filter->PEs();
    auto PPs  = simplex_trajectory_filter->PPs();

    auto PT_hessian  = PT_EE_hessians.view(0, PTs.size());
    auto PT_gradient = PT_EE_gradients.view(0, PTs.size());
    auto EE_hessian  = PT_EE_hessians.view(PTs.size(), EEs.size());
    auto EE_gradient = PT_EE_gradients.view(PTs.size(), EEs.size());

    SizeT H3x3_offset = 0;
    SizeT G3_offset   = 0;

    // PT
    {
        SizeT H3x3_count = PTs.size() * 16;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PT_hessian.size(),
                   [PT_H12x12s = PT_hessian.cviewer().name("H12x12"),
                    PTs        = PTs.cviewer().name("PTs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix12x12 H12x12 = PT_H12x12s(I);
                       Vector4i    D4     = PTs(I);
                       detail::make_spd<12>(H12x12);
                       detail::fill_contact_hessian<4>(H3x3, I, D4, H12x12);
                   });
        H3x3_offset += H3x3_count;

        SizeT G3_count = PT_gradient.size() * 4;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PT_gradient.size(),
                   [PT_G12s = PT_gradient.cviewer().name("G12"),
                    PTs     = PTs.cviewer().name("PTs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G12 = PT_G12s(I);
                       Vector4i    D4  = PTs(I);
                       detail::fill_contact_gradient<4>(G3, I, D4, G12);
                   });
        G3_offset += G3_count;
    }

    // EE
    {
        SizeT H3x3_count = EEs.size() * 16;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(EE_hessian.size(),
                   [EE_H12x12s = EE_hessian.cviewer().name("H12x12"),
                    EEs        = EEs.cviewer().name("EEs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix12x12 H12x12 = EE_H12x12s(I);
                       Vector4i    D4     = EEs(I);
                       detail::make_spd<12>(H12x12);
                       detail::fill_contact_hessian<4>(H3x3, I, D4, H12x12);
                   });

        H3x3_offset += H3x3_count;

        SizeT G3_count = EE_gradient.size() * 4;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(EE_gradient.size(),
                   [G12s = EE_gradient.cviewer().name("G12"),
                    EEs  = EEs.cviewer().name("EEs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G12 = G12s(I);
                       Vector4i    D4  = EEs(I);
                       detail::fill_contact_gradient<4>(G3, I, D4, G12);
                   });

        G3_offset += G3_count;
    }

    // PE
    {
        SizeT H3x3_count = PEs.size() * 9;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PEs.size(),
                   [PE_H9x9s = PE_hessians.cviewer().name("H9x9"),
                    PEs      = PEs.cviewer().name("PEs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix9x9 H9x9 = PE_H9x9s(I);
                       Vector3i  D3   = PEs(I);
                       detail::make_spd<9>(H9x9);
                       detail::fill_contact_hessian<3>(H3x3, I, D3, H9x9);
                   });

        H3x3_offset += H3x3_count;

        SizeT G3_count = PEs.size() * 3;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PEs.size(),
                   [G9s = PE_gradients.cviewer().name("G9"),
                    PEs = PEs.cviewer().name("PEs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G9 = G9s(I);
                       Vector3i    D3 = PEs(I);
                       detail::fill_contact_gradient<3>(G3, I, D3, G9);
                   });

        G3_offset += G3_count;
    }


    // PP
    {
        SizeT H3x3_count = PPs.size() * 4;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PPs.size(),
                   [PP_H6x6s = PP_hessians.cviewer().name("H6x6"),
                    PPs      = PPs.cviewer().name("PPs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix6x6 H6x6 = PP_H6x6s(I);
                       Vector2i  D2   = PPs(I);
                       detail::make_spd<6>(H6x6);
                       detail::fill_contact_hessian<2>(H3x3, I, D2, H6x6);
                   });

        H3x3_offset += H3x3_count;

        SizeT G3_count = PPs.size() * 2;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PPs.size(),
                   [G6s = PP_gradients.cviewer().name("G6"),
                    PPs = PPs.cviewer().name("PPs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G6 = G6s(I);
                       Vector2i    D2 = PPs(I);
                       detail::fill_contact_gradient<2>(G3, I, D2, G6);
                   });

        G3_offset += G3_count;
    }

    UIPC_ASSERT(H3x3_offset == info.hessian().triplet_count(), "size mismatch");
    UIPC_ASSERT(G3_offset == info.gradient().doublet_count(), "size mismatch");
}

//void SimplexNormalContact::Impl::classify_constraints()
//{
//    using namespace muda;
//
//    auto& filter = simplex_trajectory_filter;
//
//    auto PPs = filter->PPs();
//    auto PEs = filter->PEs();
//    auto PTs = filter->PTs();
//    auto EEs = filter->EEs();
//
//    auto positions   = global_vertex_manager->positions();
//    auto thicknesses = global_vertex_manager->thicknesses();
//    auto d_hat       = global_contact_manager->d_hat();
//    auto D_hat       = d_hat * d_hat;
//
//    // PP, PE, PT, EE all can generate the PP constraint
//    SizeT possible_PP_constraint_count =
//        PPs.size() + PEs.size() + PTs.size() + EEs.size();
//    // PE, PT, EE can generate the PT constraint
//    SizeT possible_PE_constraint_count = PEs.size() + PTs.size() + EEs.size();
//    // only PT can generate the PT constraint
//    SizeT possible_PT_constraint_count = PTs.size();
//    // only EE can generate the EE constraint
//    SizeT possible_EE_constraint_count = EEs.size();
//
//    loose_resize(temp_PP_constraints, possible_PP_constraint_count);
//    loose_resize(temp_PE_constraints, possible_PE_constraint_count);
//    loose_resize(temp_PT_constraints, possible_PT_constraint_count);
//    loose_resize(temp_EE_constraints, possible_EE_constraint_count);
//
//    SizeT PP_offset = 0;
//    SizeT PE_offset = 0;
//
//    {  // PP in PPs
//
//        auto PP_constraint_view = temp_PP_constraints.view(PP_offset, PPs.size());
//        PP_offset += PPs.size();
//
//        ParallelFor()
//            .kernel_name(__FUNCTION__)
//            .apply(PPs.size(),
//                   [PPs         = PPs.cviewer().name("PPs"),
//                    positions   = positions.viewer().name("positions"),
//                    thicknesses = thicknesses.viewer().name("thicknesses"),
//                    PP_constraints = PP_constraint_view.viewer().name("PP_constraints"),
//                    D_hat = D_hat] __device__(int I) mutable
//                   {
//                       Vector2i PP = PPs(I);
//
//                       const auto& P0 = positions(PP[0]);
//                       const auto& P1 = positions(PP[1]);
//
//                       Float D;
//                       distance::point_point_distance_unclassified(P0, P1, D);
//
//                       Float thickness =
//                           PP_thickness(thicknesses(PP[0]), thicknesses(PP[1]));
//                       Vector2i range = D_range(thickness, D_hat);
//
//                       PP_constraints(I) =
//                           is_active_D(range, D) ?
//                               SimplexContactConstraint::PP(PP[0], PP[1]) :
//                               SimplexContactConstraint::None();
//                   });
//    }
//
//    {  // PP, PE in PEs
//
//        auto PE_constraint_view = temp_PE_constraints.view(PE_offset, PEs.size());
//        PE_offset += PEs.size();
//        auto PP_constraint_view = temp_PP_constraints.view(PP_offset, PEs.size());
//        PP_offset += PEs.size();
//
//        ParallelFor()
//            .kernel_name(__FUNCTION__)
//            .apply(PEs.size(),
//                   [PEs         = PEs.cviewer().name("PEs"),
//                    positions   = positions.viewer().name("positions"),
//                    thicknesses = thicknesses.viewer().name("thicknesses"),
//                    PE_constraints = PE_constraint_view.viewer().name("PE_constraints"),
//                    PP_constraints = PP_constraint_view.viewer().name("PP_constraints"),
//                    D_hat = D_hat] __device__(int I) mutable
//                   {
//                       Vector3i PE = PEs(I);
//
//                       const auto& P  = positions(PE[0]);
//                       const auto& E0 = positions(PE[1]);
//                       const auto& E1 = positions(PE[2]);
//
//                       auto type = distance::point_edge_distance_type(P, E0, E1);
//
//                       PE_constraints(I) = SimplexContactConstraint::None();
//                       PP_constraints(I) = SimplexContactConstraint::None();
//
//                       Float    thickness = PE_thickness(thicknesses(PE[0]),
//                                                      thicknesses(PE[1]),
//                                                      thicknesses(PE[2]));
//                       Vector2i range     = D_range(thickness, D_hat);
//
//                       switch(type)
//                       {
//                           case distance::PointEdgeDistanceType::PP_PE0: {
//                               Float D;
//                               distance::point_point_distance(P, E0, D);
//
//
//                               if(is_active_D(range, D))
//                               {
//                                   PP_constraints(I) = SimplexContactConstraint::PP_in_PE(
//                                       PE[0], PE[1], PE[2]);
//                               }
//                           }
//                           break;
//                           case distance::PointEdgeDistanceType::PP_PE1: {
//                               Float D;
//                               distance::point_point_distance(P, E1, D);
//
//                               if(is_active_D(range, D))
//                               {
//                                   PP_constraints(I) = SimplexContactConstraint::PP_in_PE(
//                                       PE[0], PE[2], PE[1]);
//                               }
//                           }
//                           break;
//                           case distance::PointEdgeDistanceType::PE: {
//                               Float D;
//                               distance::point_edge_distance(P, E0, E1, D);
//
//                               if(is_active_D(range, D))
//                               {
//                                   PE_constraints(I) = SimplexContactConstraint::PE(
//                                       PE[0], PE.segment<2>(1));
//                               }
//                           }
//                           break;
//                           default: {
//                               MUDA_ERROR_WITH_LOCATION("Invalid Type");
//                           }
//                           break;
//                       }
//                   });
//    }
//
//    {  // PT, PE, PP in PTs
//
//        auto PT_constraint_view = temp_PT_constraints.view(0, PTs.size());
//        auto PE_constraint_view = temp_PE_constraints.view(PE_offset, PTs.size());
//        PE_offset += PTs.size();
//        auto PP_constraint_view = temp_PP_constraints.view(PP_offset, PTs.size());
//        PP_offset += PTs.size();
//
//        ParallelFor()
//            .kernel_name(__FUNCTION__)
//            .apply(
//                PTs.size(),
//                [PTs         = PTs.cviewer().name("PTs"),
//                 positions   = positions.viewer().name("positions"),
//                 thicknesses = thicknesses.viewer().name("thicknesses"),
//                 PT_constraints = PT_constraint_view.viewer().name("PT_constraints"),
//                 PE_constraints = PE_constraint_view.viewer().name("PE_constraints"),
//                 PP_constraints = PP_constraint_view.viewer().name("PP_constraints"),
//                 D_hat = D_hat] __device__(int I) mutable
//                {
//                    Vector4i PT = PTs(I);
//
//                    const auto& P  = positions(PT[0]);
//                    const auto& T0 = positions(PT[1]);
//                    const auto& T1 = positions(PT[2]);
//                    const auto& T2 = positions(PT[3]);
//
//                    auto type = distance::point_triangle_distance_type(P, T0, T1, T2);
//
//                    PT_constraints(I) = SimplexContactConstraint::None();
//                    PE_constraints(I) = SimplexContactConstraint::None();
//                    PP_constraints(I) = SimplexContactConstraint::None();
//
//                    Float    thickness = PT_thickness(thicknesses(PT[0]),
//                                                   thicknesses(PT[1]),
//                                                   thicknesses(PT[2]),
//                                                   thicknesses(PT[3]));
//                    Vector2i range     = D_range(thickness, D_hat);
//
//                    switch(type)
//                    {
//                        case distance::PointTriangleDistanceType::PP_PT0: {
//                            Float D;
//                            distance::point_point_distance(P, T0, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_PT(PT[0],  // P
//                                                                       PT[1],  // P
//                                                                       {PT[2], PT[3]}  // inactive E
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::PointTriangleDistanceType::PP_PT1: {
//                            Float D;
//                            distance::point_point_distance(P, T1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_PT(PT[0],  // P
//                                                                       PT[2],  // P
//                                                                       {PT[1], PT[3]}  // inactive E
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::PointTriangleDistanceType::PP_PT2: {
//                            Float D;
//                            distance::point_point_distance(P, T2, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_PT(PT[0],  // P
//                                                                       PT[3],  // P
//                                                                       {PT[1], PT[2]}  // inactive E
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::PointTriangleDistanceType::PE_PT0T1: {
//                            Float D;
//                            distance::point_edge_distance(P, T0, T1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) =
//                                    SimplexContactConstraint::PE_in_PT(PT[0],  // P
//                                                                       {PT[1], PT[2]},  // E
//                                                                       PT[3]  // inactive P
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::PointTriangleDistanceType::PE_PT1T2: {
//                            Float D;
//                            distance::point_edge_distance(P, T1, T2, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) =
//                                    SimplexContactConstraint::PE_in_PT(PT[0],  // P
//                                                                       {PT[2], PT[3]},  // E
//                                                                       PT[1]  // inactive P
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::PointTriangleDistanceType::PE_PT2T0: {
//                            Float D;
//                            distance::point_edge_distance(P, T2, T0, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) =
//                                    SimplexContactConstraint::PE_in_PT(PT[0],  // P
//                                                                       {PT[3], PT[1]},  // E
//                                                                       PT[2]  // inactive P
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::PointTriangleDistanceType::PT: {
//                            Float D;
//                            distance::point_triangle_distance(P, T0, T1, T2, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PT_constraints(I) =
//                                    SimplexContactConstraint::PT(PT[0], PT.segment<3>(1));
//                            }
//                        }
//                        break;
//                        default: {
//                            MUDA_ERROR_WITH_LOCATION("Invalid Type");
//                        }
//                        break;
//                    }
//                });
//    }
//
//    {  // EE, PE, PP in EEs
//
//        auto EE_constraint_view = temp_EE_constraints.view(0, EEs.size());
//        auto PE_constraint_view = temp_PE_constraints.view(PE_offset, EEs.size());
//        PE_offset += EEs.size();
//        auto PP_constraint_view = temp_PP_constraints.view(PP_offset, EEs.size());
//        PP_offset += EEs.size();
//
//        ParallelFor()
//            .kernel_name(__FUNCTION__)
//            .apply(
//                EEs.size(),
//                [EEs         = EEs.cviewer().name("EEs"),
//                 positions   = positions.viewer().name("positions"),
//                 thicknesses = thicknesses.viewer().name("thicknesses"),
//                 EE_constraints = EE_constraint_view.viewer().name("EE_constraints"),
//                 PE_constraints = PE_constraint_view.viewer().name("PE_constraints"),
//                 PP_constraints = PP_constraint_view.viewer().name("PP_constraints"),
//                 D_hat = D_hat] __device__(int I) mutable
//                {
//                    Vector4i EE = EEs(I);
//
//                    const auto& Ea0 = positions(EE[0]);
//                    const auto& Ea1 = positions(EE[1]);
//                    const auto& Eb0 = positions(EE[2]);
//                    const auto& Eb1 = positions(EE[3]);
//
//                    auto type = distance::edge_edge_distance_type(Ea0, Ea1, Eb0, Eb1);
//
//                    EE_constraints(I) = SimplexContactConstraint::None();
//                    PE_constraints(I) = SimplexContactConstraint::None();
//                    PP_constraints(I) = SimplexContactConstraint::None();
//
//                    Float    thickness = EE_thickness(thicknesses(EE[0]),
//                                                   thicknesses(EE[1]),
//                                                   thicknesses(EE[2]),
//                                                   thicknesses(EE[3]));
//                    Vector2i range     = D_range(thickness, D_hat);
//
//                    switch(type)
//                    {
//                        case distance::EdgeEdgeDistanceType::PP_Ea0Eb0: {
//                            Float D;
//                            distance::point_point_distance(Ea0, Ea1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_EE(EE[0],  // Pa
//                                                                       EE[2],  // Pb
//                                                                       EE[1],  // inactive Pa
//                                                                       EE[3]  // inactive Pb
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::PP_Ea0Eb1: {
//                            Float D;
//                            distance::point_point_distance(Ea0, Ea1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_EE(EE[0],  // Pa
//                                                                       EE[3],  // Pb
//                                                                       EE[1],  // inactive Pa
//                                                                       EE[2]  // inactive Pb
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::PP_Ea1Eb0: {
//                            Float D;
//                            distance::point_point_distance(Ea0, Ea1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_EE(EE[1],  // Pa
//                                                                       EE[2],  // Pb
//                                                                       EE[0],  // inactive Pa
//                                                                       EE[3]  // inactive Pb
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::PP_Ea1Eb1: {
//                            Float D;
//                            distance::point_point_distance(Ea0, Ea1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PP_constraints(I) =
//                                    SimplexContactConstraint::PP_in_EE(EE[1],  // Pa
//                                                                       EE[3],  // Pb
//                                                                       EE[0],  // inactive Pa
//                                                                       EE[2]  // inactive Pb
//                                    );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::PE_Ea0Eb0Eb1: {
//                            Float D;
//                            distance::point_edge_distance(Ea0, Ea1, Eb0, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) = SimplexContactConstraint::PE_in_EE(
//                                    EE[0],             // P
//                                    EE.segment<2>(2),  // E
//                                    EE[1]              // inactive P
//                                );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::PE_Ea1Eb0Eb1: {
//                            Float D;
//                            distance::point_edge_distance(Ea1, Ea0, Eb0, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) = SimplexContactConstraint::PE_in_EE(
//                                    EE[1],             // P
//                                    EE.segment<2>(2),  // E
//                                    EE[0]              // inactive P
//                                );
//                            }
//                        }
//                        case distance::EdgeEdgeDistanceType::PE_Eb0Ea0Ea1: {
//                            Float D;
//                            distance::point_edge_distance(Eb0, Eb1, Ea0, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) = SimplexContactConstraint::PE_in_EE(
//                                    EE[2],             // P
//                                    EE.segment<2>(0),  // E
//                                    EE[3]              // inactive P
//                                );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::PE_Eb1Ea0Ea1: {
//                            Float D;
//                            distance::point_edge_distance(Eb1, Eb0, Ea0, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                PE_constraints(I) = SimplexContactConstraint::PE_in_EE(
//                                    EE[3],             // P
//                                    EE.segment<2>(0),  // E
//                                    EE[2]              // inactive P
//                                );
//                            }
//                        }
//                        break;
//                        case distance::EdgeEdgeDistanceType::EE: {
//                            Float D;
//                            distance::edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);
//
//                            if(is_active_D(range, D))
//                            {
//                                EE_constraints(I) = SimplexContactConstraint::EE(
//                                    EE.segment<2>(0), EE.segment<2>(2));
//                            }
//                        }
//                    }
//                });
//    }
//
//    UIPC_ASSERT(PP_offset == temp_PP_constraints.size(), "size mismatch");
//    UIPC_ASSERT(PE_offset == temp_PE_constraints.size(), "size mismatch");
//
//    {  // PP constraints
//        DeviceSelect().If(temp_PP_constraints.data(),
//                          PP_constraints.data(),
//                          selected_count.data(),
//                          temp_PP_constraints.size(),
//                          [] CUB_RUNTIME_FUNCTION(const SimplexContactConstraint& c)
//                          { return !c.is_none(); });
//
//        IndexT h_selected = selected_count;
//        PP_constraints.resize(h_selected);
//    }
//
//    {  // PE constraints
//        DeviceSelect().If(temp_PE_constraints.data(),
//                          PE_constraints.data(),
//                          selected_count.data(),
//                          temp_PE_constraints.size(),
//                          [] CUB_RUNTIME_FUNCTION(const SimplexContactConstraint& c)
//                          { return !c.is_none(); });
//
//        IndexT h_selected = selected_count;
//
//        PE_constraints.resize(h_selected);
//    }
//
//    {  // PT constraints
//        DeviceSelect().If(temp_PT_constraints.data(),
//                          PT_constraints.data(),
//                          selected_count.data(),
//                          temp_PT_constraints.size(),
//                          [] CUB_RUNTIME_FUNCTION(const SimplexContactConstraint& c)
//                          { return !c.is_none(); });
//
//        IndexT h_selected = selected_count;
//        PT_constraints.resize(h_selected);
//    }
//
//    {  // EE constraints
//        DeviceSelect().If(temp_EE_constraints.data(),
//                          EE_constraints.data(),
//                          selected_count.data(),
//                          temp_EE_constraints.size(),
//                          [] CUB_RUNTIME_FUNCTION(const SimplexContactConstraint& c)
//                          { return !c.is_none(); });
//
//        IndexT h_selected = selected_count;
//        EE_constraints.resize(h_selected);
//    }
//}
}  // namespace uipc::backend::cuda
