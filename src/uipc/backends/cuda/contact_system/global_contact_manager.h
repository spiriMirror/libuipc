#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
namespace uipc::backend::cuda
{
class GlobalContactManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class Impl
    {
      public:
        void                 compute_d_hat();
        void                 compute_adaptive_kappa();
        GlobalVertexManager* global_vertex_manager = nullptr;

        Float d_hat         = std::numeric_limits<Float>::infinity();
        Float related_d_hat = std::numeric_limits<Float>::infinity();
        Float kappa         = std::numeric_limits<Float>::infinity();
    };

    Float d_hat() const;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    void compute_d_hat();
    void compute_contact();
    void compute_adaptive_kappa();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda