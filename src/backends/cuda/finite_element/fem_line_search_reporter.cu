#include <finite_element/fem_line_search_reporter.h>
#include <finite_element/fem_line_search_subreporter.h>
#include <finite_element/finite_element_kinetic.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FEMLineSearchReporter_step_forward_kernel(
        cuda_tool::CBufferView<Vector3> x_temps,
        cuda_tool::BufferView<Vector3>  xs,
        cuda_tool::CBufferView<Vector3> dxs,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        xs(i) = x_temps(i) + alpha * dxs(i);
    }

    __global__ void FEMLineSearchReporter_combine_energy_kernel(
        cuda_tool::CVarView<Float> kinetic_energy,
        cuda_tool::CVarView<Float> reporter_energy,
        cuda_tool::VarView<Float>  total_energy)
    {
        if(blockIdx.x != 0 || threadIdx.x != 0)
            return;
        *total_energy = *kinetic_energy + *reporter_energy;
    }
}  // namespace

REGISTER_SIM_SYSTEM(FEMLineSearchReporter);

void FEMLineSearchReporter::do_init(InitInfo& info)
{
    m_impl.init(info);
}

void FEMLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();
}

void FEMLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    m_impl.record_start_point(info);
}

void FEMLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    m_impl.step_forward(info);
}

void FEMLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void FEMLineSearchReporter::Impl::record_start_point(LineSearcher::RecordInfo& info)
{
    using namespace cuda_tool;

    fem().x_temps = fem().xs;
}

void FEMLineSearchReporter::Impl::step_forward(LineSearcher::StepInfo& info)
{
    auto k = FEMLineSearchReporter_step_forward_kernel;
    int  n = (int)fem().xs.size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            fem().x_temps.cview(), fem().xs.view(), fem().dxs.cview(), info.alpha, n);
    }
}

void FEMLineSearchReporter::Impl::compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace cuda_tool;

    // Compute Kinetic (special)
    {
        auto vertex_count = fem().xs.size();
        kinetic_energies.resize_discard(vertex_count);
        auto kinetic_info = ComputeEnergyInfo{kinetic_energies.view(), info.dt()};
        finite_element_kinetic->compute_energy(kinetic_info);

        DeviceReduce().Sum(kinetic_energies.data(),
                           total_kinetic_energy.data(),
                           kinetic_energies.size());
    }

    // Collect the energy from all reporters
    {
        auto         reporter_view = reporters.view();
        span<IndexT> counts        = reporter_energy_offsets_counts.counts();
        for(auto&& [i, R] : enumerate(reporter_view))
        {
            ReportExtentInfo this_info;
            R->report_extent(this_info);
            counts[i] = this_info.m_energy_count;
        }

        reporter_energy_offsets_counts.scan();
        reporter_energies.resize_discard(reporter_energy_offsets_counts.total_count());

        for(auto&& [i, R] : enumerate(reporter_view))
        {
            auto [offset, count] = reporter_energy_offsets_counts[i];
            auto this_info =
                ComputeEnergyInfo{reporter_energies.view(offset, count), info.dt()};
            R->compute_energy(this_info);
        }

        DeviceReduce().Sum(reporter_energies.data(),
                           total_reporter_energy.data(),
                           reporter_energies.size());
    }

    FEMLineSearchReporter_combine_energy_kernel<<<1, 1>>>(
        total_kinetic_energy.cview(), total_reporter_energy.cview(), info.energy());
}

void FEMLineSearchReporter::Impl::init(LineSearchReporter::InitInfo& info)
{
    kinetic_energies.resize_discard(fem().xs.size());

    auto reporter_view = reporters.view();
    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;
    for(auto&& [i, R] : enumerate(reporter_view))
        R->init();

    reporter_energy_offsets_counts.resize(reporter_view.size());
}

void FEMLineSearchReporter::add_reporter(FEMLineSearchSubreporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter is null");
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.reporters.register_sim_system(*reporter);
}

void FEMLineSearchReporter::add_kinetic(FiniteElementKinetic* kinetic)
{
    UIPC_ASSERT(kinetic, "kinetic is null");
    check_state(SimEngineState::BuildSystems, "add_kinetic()");
    m_impl.finite_element_kinetic.register_sim_system(*kinetic);
}
}  // namespace uipc::backend::cuda
