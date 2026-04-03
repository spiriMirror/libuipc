#include <sanity_check/sanity_checker.h>
#include <finite_element/finite_element_method.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class SimplicialVolumeCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementMethod> fem;
        SimSystemSlot<AffineBodyDynamics>  abd;

        muda::DeviceVar<IndexT> violation_count;

        void check(CheckInfo& info);
    };

  protected:
    void do_build(BuildInfo& info) override
    {
        m_impl.fem = find<FiniteElementMethod>();
        m_impl.abd = find<AffineBodyDynamics>();

        if(!m_impl.fem && !m_impl.abd)
            throw SimSystemException("Neither FEM nor ABD found");
    }

    void do_init(InitInfo& info) override {}
    void do_check(CheckInfo& info) override { m_impl.check(info); }

  private:
    Impl m_impl;
};

void SimplicialVolumeCheck::Impl::check(CheckInfo& info)
{
    using namespace muda;

    violation_count = 0;

    // ============================================================
    // Check FEM tetrahedra volumes
    // ============================================================
    if(fem)
    {
        auto tets = fem->tets();
        auto xs   = fem->xs();

        if(tets.size() > 0)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(tets.size(),
                       [tets = tets.viewer().name("tets"),
                        xs   = xs.viewer().name("xs"),
                        violation = violation_count.viewer().name("violation")] __device__(int i) mutable
                       {
                           auto T = tets(i);

                           Vector3 e1 = xs(T[1]) - xs(T[0]);
                           Vector3 e2 = xs(T[2]) - xs(T[0]);
                           Vector3 e3 = xs(T[3]) - xs(T[0]);

                           // signed volume = (1/6) * (e1 x e2) · e3
                           Float vol = e1.cross(e2).dot(e3) / 6.0;

                           if(vol <= 0.0)
                               atomicAdd(violation.data(), IndexT(1));
                       });
        }
    }

    // ============================================================
    // Check AffineBody volumes
    // ============================================================
    if(abd)
    {
        auto body_volumes = abd->body_volumes();

        if(body_volumes.size() > 0)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(body_volumes.size(),
                       [volumes = body_volumes.viewer().name("volumes"),
                        violation = violation_count.viewer().name("violation")] __device__(int i) mutable
                       {
                           if(volumes(i) <= 0.0)
                               atomicAdd(violation.data(), IndexT(1));
                       });
        }
    }

    // ============================================================
    // Report
    // ============================================================
    IndexT h_violation = violation_count;
    if(h_violation > 0)
    {
        logger::error(
            "GPU SanityCheck: {} non-positive volume(s) detected "
            "(frame={}, newton={})",
            h_violation,
            info.frame(),
            info.newton_iter());
    }
}

REGISTER_SIM_SYSTEM(SimplicialVolumeCheck);
}  // namespace uipc::backend::cuda
