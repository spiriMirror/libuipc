#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_1d_constitution_diff_parm_reporter.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <numbers>
#include <utils/matrix_assembly_utils.h>
#include <utils/matrix_unpacker.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
// Constitution UID by libuipc specification
static constexpr U64 ConstitutionUID = 12ull;

class HookeanSpring1D final : public Codim1DConstitution
{
  public:
    using Codim1DConstitution::Codim1DConstitution;

    vector<Float>             h_kappas;
    muda::DeviceBuffer<Float> kappas;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_kappas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto kappa = sc.edges().find<Float>("kappa");
                return kappa->view();
            },
            [&](const ForEachInfo& I, Float kappa)
            {
                auto vI = I.global_index();
                // retrieve material parameters
                h_kappas[vI] = kappa;
            });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace NS = sym::hookean_spring_1d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [kappas = kappas.cviewer().name("kappas"),
                    rest_lengths = info.rest_lengths().viewer().name("rest_lengths"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    energies = info.energies().viewer().name("energies"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    x_bars   = info.x_bars().viewer().name("x_bars"),
                    dt       = info.dt(),
                    Pi       = std::numbers::pi] __device__(int I)
                   {
                       Vector6  X;
                       Vector2i idx = indices(I);
                       for(int i = 0; i < 2; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Float L0 = rest_lengths(I);
                       Float r =
                           edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));
                       Float kappa = kappas(I);

                       Float Vdt2 = L0 * r * r * Pi * dt * dt;

                       Float E;
                       NS::E(E, kappa, X, L0);
                       energies(I) = E * Vdt2;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace NS = sym::hookean_spring_1d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [G3s    = info.gradients().viewer().name("gradients"),
                    H3x3s  = info.hessians().viewer().name("hessians"),
                    kappas = kappas.cviewer().name("kappas"),
                    rest_lengths = info.rest_lengths().viewer().name("rest_lengths"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    x_bars   = info.x_bars().viewer().name("x_bars"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed"),
                    dt       = info.dt(),
                    Pi       = std::numbers::pi] __device__(int I) mutable
                   {
                       Vector6  X;
                       Vector2i idx = indices(I);
                       for(int i = 0; i < 2; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Float L0 = rest_lengths(I);
                       Float r =
                           edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));
                       Float kappa = kappas(I);

                       Float Vdt2 = L0 * r * r * Pi * dt * dt;

                       Vector2i ignore = {is_fixed(idx(0)), is_fixed(idx(1))};

                       Vector6 G;
                       NS::dEdX(G, kappa, X, L0);
                       G *= Vdt2;
                       DoubletVectorAssembler VA{G3s};
                       VA.segment<2>(I * 2).write(idx, ignore, G);

                       Matrix6x6 H;
                       NS::ddEddX(H, kappa, X, L0);
                       H *= Vdt2;
                       make_spd(H);
                       TripletMatrixAssembler MA{H3x3s};
                       MA.block<2, 2>(I * 2 * 2).write(idx, ignore, H);
                   });
    }
};

class DiffHookeanSpring1D final : public Codim1DConstitutionDiffParmReporter
{
  public:
    using Base = Codim1DConstitutionDiffParmReporter;
    using Base::Base;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    vector<IndexT> h_diff_parm_indices;
    vector<IndexT> h_edge_indices;

    muda::DeviceBuffer<IndexT> diff_parm_indices;  // I -> diff_parm_index
    muda::DeviceBuffer<IndexT> edge_indices;       // I -> edge_index


    HookeanSpring1D& constitution() noexcept
    {
        return dynamic_cast<HookeanSpring1D&>(Base::constitution());
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto prim_count = info.primitive_count();


        h_diff_parm_indices.reserve(prim_count);
        h_edge_indices.reserve(prim_count);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto diff_kappa = sc.edges().find<IndexT>("diff/kappa");
                return diff_kappa->view();
            },
            [&](const ForEachInfo& I, IndexT diff_kappa)
            {
                if(diff_kappa >= 0)
                {
                    h_diff_parm_indices.push_back(diff_kappa);
                    h_edge_indices.push_back(I.global_index());
                }
            });

        diff_parm_indices.resize(h_diff_parm_indices.size());
        diff_parm_indices.view().copy_from(h_diff_parm_indices.data());

        edge_indices.resize(h_edge_indices.size());
        edge_indices.view().copy_from(h_edge_indices.data());
    }

    virtual void do_update(DiffParmUpdateInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(diff_parm_indices.size(),
                   [parameters = info.parameters().viewer().name("parameters"),
                    edge_indices = edge_indices.cviewer().name("edge_indices"),
                    kappas = constitution().kappas.viewer().name("kappas"),
                    diff_parm_indices = diff_parm_indices.cviewer().name(
                        "diff_parm_indices")] __device__(int I) mutable
                   {
                       // Get the data from the parameters and update the kappa
                       auto& kappa = kappas(edge_indices(I));
                       kappa       = parameters(diff_parm_indices(I));
                   });
    }


    virtual void do_report_extent(DiffParmExtentInfo& info) override
    {
        // every kappa related to 2 vertices, which is 6 dof
        auto stencil_dim   = 2;
        auto dof           = stencil_dim * 3;
        auto parm_count    = diff_parm_indices.size();
        auto triplet_count = (dof * 1) * parm_count;

        info.triplet_count(triplet_count);
    }

    virtual void do_assemble(DiffParmInfo& info) override
    {
        using namespace muda;
        namespace NS = sym::hookean_spring_1d;

        auto pGpP = info.pGpP();

        auto N = diff_parm_indices.size();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [diff_parm_indices = diff_parm_indices.cviewer().name("diff_parm_indices"),
                    edge_indices = edge_indices.cviewer().name("edge_indices"),
                    pGpP         = pGpP.viewer().name("pGpP"),
                    kappas = constitution().kappas.viewer().name("kappas"),
                    rest_lengths = info.rest_lengths().viewer().name("rest_lengths"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    x_bars   = info.x_bars().viewer().name("x_bars"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed"),
                    dt       = info.dt(),
                    Pi       = std::numbers::pi] __device__(int I) mutable
                   {
                       auto edge_index = edge_indices(I);
                       auto parm_index = diff_parm_indices(I);

                       Vector6  X;
                       Vector2i idx = indices(edge_index);
                       for(int k = 0; k < 2; ++k)
                           X.segment<3>(3 * k) = xs(idx(k));

                       Float L0 = rest_lengths(edge_index);
                       Float r =
                           edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));

                       Float kappa = kappas(edge_index);

                       Float Vdt2 = L0 * r * r * Pi * dt * dt;

                       Vector6 pGpk;
                       NS::pGpk(pGpk, kappa, X, L0);
                       pGpk *= Vdt2;

                       TripletMatrixUnpacker unpacker(pGpP);

                       Vector2i ignore = {is_fixed(idx(0)), is_fixed(idx(1))};

                       zero_out(pGpk, ignore);

                       // cout << "pGpk: " << pGpk.transpose().eval() << "\n";

                       for(int k = 0; k < 2; ++k)
                       {
                           auto i = idx(k) * 3;  // 3 dof per vertex
                           auto j = parm_index;

                           unpacker
                               .segment<3>((I + k) * 3)  // take a range for 3 dof, [(I+k)*3, (I+k)*3+3)
                               .write(i,
                                      j,
                                      pGpk.segment<3>(3 * k)  // 3x1 vector
                               );
                       }
                   });
    }
};

REGISTER_SIM_SYSTEM(HookeanSpring1D);
REGISTER_SIM_SYSTEM(DiffHookeanSpring1D);
}  // namespace uipc::backend::cuda
