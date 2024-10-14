#include <finite_element/finite_element_energy_producer.h>

namespace uipc::backend::cuda
{
void FiniteElementEnergyProducer::do_build()
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();

    BuildInfo info;
    do_build(info);
}

void FiniteElementEnergyProducer::collect_extent_info()
{
    ReportExtentInfo info;
    do_report_extent(info);

    SizeT N = info.m_energy_count;
    SizeT D = info.m_stencil_dim;

    m_impl.stencil_dim    = D;
    m_impl.energy_count   = N;
    m_impl.gradient_count = N * D;
    m_impl.hessian_count  = N * D * D;
}

void FiniteElementEnergyProducer::compute_energy()
{
    auto global_energies =
        m_impl.finite_element_method->m_impl.energy_producer_energies.view();
    ComputeEnergyInfo info;
    info.m_energies = global_energies.subview(m_impl.energy_offset, m_impl.energy_count);
    do_compute_energy(info);
}

void FiniteElementEnergyProducer::assemble_gradient_hessian(AssemblyInfo& info)
{
    ComputeGradientHessianInfo this_info;
    auto                       global_gradient_view =
        m_impl.finite_element_method->m_impl.energy_producer_gradients.view();

    this_info.m_dt = info.dt;

    this_info.m_gradients =
        global_gradient_view.subview(m_impl.gradient_offset, m_impl.gradient_count);

    this_info.m_hessians =
        info.hessians.subview(m_impl.hessian_offset, m_impl.hessian_count);

    Vector2i vertex_offset_count = get_vertex_offset_count();
    if(vertex_offset_count[1] > 0)  // basic constitution has vertex range
    {
        this_info.m_gradients =
            this_info.m_gradients.subvector(vertex_offset_count[0],  // offset
                                            vertex_offset_count[1]   // extent
            );

        this_info.m_hessians = this_info.m_hessians.submatrix(
            {vertex_offset_count[0], vertex_offset_count[0]},  // offset
            {vertex_offset_count[1], vertex_offset_count[1]}   // extent
        );
    }
    // extra constitution has no vertex range (can connect any vertices)

    do_compute_gradient_hessian(this_info);
}
}  // namespace uipc::backend::cuda
