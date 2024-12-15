#pragma once
#include <type_define.h>
#include <sstream>
#include <sim_engine_state.h>
#include <backends/common/sim_engine.h>
#include <sim_action_collection.h>

namespace uipc::backend::cuda
{
class GlobalVertexManager;
class GlobalSimpicialSurfaceManager;
class GlobalContactManager;
class GlobalTrajectoryFilter;

class DofPredictor;
class LineSearcher;
class GradientHessianComputer;
class GlobalLinearSystem;
class GlobalAnimator;
class GlobalDiffSimManager;

class SimEngine final : public backend::SimEngine
{
    class DeviceImpl;
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
    virtual void  do_backward() override;
    virtual SizeT get_frame() const override;

    virtual bool do_dump(DumpInfo&) override;
    virtual bool do_try_recover(RecoverInfo&) override;
    virtual void do_apply_recover(RecoverInfo&) override;
    virtual void do_clear_recover(RecoverInfo&) override;

    void build();
    void init_scene();
    void dump_global_surface(std::string_view name);

    DeviceImpl&       device_impl() noexcept;
    U<DeviceImpl>     m_device_impl;
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
    GlobalSimpicialSurfaceManager* m_global_simplicial_surface_manager = nullptr;
    GlobalContactManager*   m_global_contact_manager   = nullptr;
    GlobalTrajectoryFilter* m_global_trajectory_filter = nullptr;

    DofPredictor*            m_dof_predictor             = nullptr;
    LineSearcher*            m_line_searcher             = nullptr;
    GradientHessianComputer* m_gradient_hessian_computer = nullptr;
    GlobalLinearSystem*      m_global_linear_system      = nullptr;
    GlobalAnimator*          m_global_animator           = nullptr;
    GlobalDiffSimManager*    m_global_diff_sim_manager   = nullptr;

    Float m_abs_tol             = 0.0;
    Float m_newton_velocity_tol = 0.01;
    Float m_newton_scene_tol    = 0.01;
    SizeT m_newton_max_iter     = 1000;
    SizeT m_current_frame       = 0;
    bool  m_friction_enabled    = false;
    SizeT m_last_solved_frame   = 0;
    bool  m_strict_mode         = false;
    Float m_ccd_tol             = 1;
};
}  // namespace uipc::backend::cuda