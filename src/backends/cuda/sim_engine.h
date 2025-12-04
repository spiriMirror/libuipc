#pragma once
#include <type_define.h>
#include <sstream>
#include <sim_engine_state.h>
#include <backends/common/sim_engine.h>
#include <sim_action_collection.h>

namespace uipc::backend::cuda
{
class GlobalVertexManager;
class GlobalSimplicialSurfaceManager;
class GlobalBodyManager;
class GlobalContactManager;
class GlobalDyTopoEffectManager;
class GlobalTrajectoryFilter;

class TimeIntegratorManager;
class LineSearcher;
class GlobalLinearSystem;
class GlobalAnimator;
class GlobalExternalForceManager;
class GlobalDiffSimManager;
class AffineBodyDynamics;
class FiniteElementMethod;
class InterAffineBodyConstitutionManager;
class NewtonToleranceManager;


class SimEngine final : public backend::SimEngine
{
    friend class SimSystem;

  public:
    SimEngine(EngineCreateInfo*);
    virtual ~SimEngine();

    SimEngine(const SimEngine&)            = delete;
    SimEngine& operator=(const SimEngine&) = delete;

    SimEngineState state() const noexcept;

  private:
    virtual void  do_init(InitInfo& info) override;
    virtual void  do_advance() override;
    virtual void  do_sync() override;
    virtual void  do_retrieve() override;
    virtual SizeT get_frame() const override;

    virtual bool do_dump(DumpInfo&) override;
    virtual bool do_try_recover(RecoverInfo&) override;
    virtual void do_apply_recover(RecoverInfo&) override;
    virtual void do_clear_recover(RecoverInfo&) override;

    void build();
    void init_scene();
    void dump_global_surface(std::string_view name);

    std::stringstream m_string_stream;
    SimEngineState    m_state = SimEngineState::None;

    // Events
    SimActionCollection<void()> m_on_init_scene;
    void                        event_init_scene();
    SimActionCollection<void()> m_on_rebuild_scene;
    void                        event_rebuild_scene();
    SimActionCollection<void()> m_on_write_scene;
    void                        event_write_scene();


  private:
    // Aware Top Systems

    GlobalVertexManager* m_global_vertex_manager = nullptr;
    GlobalSimplicialSurfaceManager* m_global_simplicial_surface_manager = nullptr;
    GlobalBodyManager*         m_global_body_manager          = nullptr;
    GlobalContactManager*      m_global_contact_manager       = nullptr;
    GlobalDyTopoEffectManager* m_global_dytopo_effect_manager = nullptr;
    GlobalTrajectoryFilter*    m_global_trajectory_filter     = nullptr;

    // Newton Solver Systems
    TimeIntegratorManager*  m_time_integrator_manager  = nullptr;
    LineSearcher*           m_line_searcher            = nullptr;
    GlobalLinearSystem*     m_global_linear_system     = nullptr;
    NewtonToleranceManager* m_newton_tolerance_manager = nullptr;

    GlobalAnimator*              m_global_animator               = nullptr;
    GlobalExternalForceManager*  m_global_external_force_manager = nullptr;
    GlobalDiffSimManager*        m_global_diff_sim_manager       = nullptr;
    //GlobalDiffContactManager*    m_global_diff_contact_manager    = nullptr;
    //GlobalAdjointMethodReplayer* m_global_adjoint_method_replayer = nullptr;
    AffineBodyDynamics* m_affine_body_dynamics = nullptr;
    InterAffineBodyConstitutionManager* m_inter_affine_body_constitution_manager = nullptr;
    //ABDDiffSimManager*           m_abd_diff_sim_manager           = nullptr;
    FiniteElementMethod* m_finite_element_method = nullptr;


    bool  m_friction_enabled = false;
    SizeT m_current_frame    = 0;
    Float m_newton_scene_tol = 0.01;

    template <typename T>
    using CAS = S<const geometry::AttributeSlot<T>>;

    CAS<Float>  m_newton_velocity_tol;
    CAS<IndexT> m_newton_max_iter;
    CAS<IndexT> m_newton_min_iter;
    CAS<IndexT> m_strict_mode;
    CAS<Float>  m_ccd_tol;
    CAS<IndexT> m_dump_surface;
};
}  // namespace uipc::backend::cuda