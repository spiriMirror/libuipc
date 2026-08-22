#pragma once

#include <sim_system.h>
#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>

namespace uipc::backend::cuda
{

class ActiveSetReporter;
class ALStiffnessEstimator;
class GlobalVertexManager;

class GlobalActiveSetManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class NonPenetratePositionInfo
    {
      public:
        NonPenetratePositionInfo(Impl* impl, SizeT offset, SizeT count) noexcept;
        cuda_tool::BufferView<Vector3> non_penetrate_positions() const noexcept;

      private:
        friend class GlobalActiveSetManager;
        Impl* m_impl;
        SizeT m_offset;
        SizeT m_count;
    };

    class StiffnessEstimateInfo
    {
      public:
        StiffnessEstimateInfo(Impl* impl) noexcept;
        cuda_tool::BufferView<Float> mu_vertices(SizeT offset, SizeT count) const noexcept;
        Float dt() const noexcept;

      private:
        friend class GlobalActiveSetManager;
        Impl* m_impl;
    };

    class Impl
    {
      public:
        void init(WorldVisitor& world);

        SimSystemSlot<GlobalVertexManager> global_vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager> global_simplicial_surface_manager;
        SimSystemSlot<GlobalTrajectoryFilter>  global_trajectory_filter;
        SimSystemSlot<SimplexTrajectoryFilter> simplex_trajectory_filter;
        SimSystemSlot<VertexHalfPlaneTrajectoryFilter> vertex_half_plane_trajectory_filter;
        SimSystemSlot<HalfPlane> half_plane;

        SimSystemSlotCollection<ActiveSetReporter>    active_set_reporters;
        SimSystemSlotCollection<ALStiffnessEstimator> stiffness_estimators;

        cuda_tool::DeviceBuffer<Float> mu_vertices;

        cuda_tool::DeviceBuffer<Vector2i> PH_idx;
        cuda_tool::DeviceBuffer<Float>    PH_lambda;
        cuda_tool::DeviceBuffer<int>      PH_cnt;

        cuda_tool::DeviceBuffer<int>     PHs;
        cuda_tool::DeviceBuffer<Float>   PH_d0, PH_slack;
        cuda_tool::DeviceBuffer<Vector3> PH_d_grad;

        cuda_tool::DeviceBuffer<Vector2i> PHs_friction;
        cuda_tool::DeviceBuffer<Float>    PH_lambda_friction;

        cuda_tool::DeviceBuffer<Vector2i> PT_idx;
        cuda_tool::DeviceBuffer<Float>    PT_lambda;
        cuda_tool::DeviceBuffer<int>      PT_cnt;

        cuda_tool::DeviceBuffer<Vector4i> PTs;
        cuda_tool::DeviceBuffer<Float>    PT_d0, PT_slack;
        cuda_tool::DeviceBuffer<Vector12> PT_d_grad;

        cuda_tool::DeviceBuffer<Vector4i> PTs_friction;
        cuda_tool::DeviceBuffer<Float>    PT_lambda_friction;

        cuda_tool::DeviceBuffer<Vector2i> EE_idx;
        cuda_tool::DeviceBuffer<Float>    EE_lambda;
        cuda_tool::DeviceBuffer<int>      EE_cnt;

        cuda_tool::DeviceBuffer<Vector4i> EEs;
        cuda_tool::DeviceBuffer<Float>    EE_d0, EE_slack;
        cuda_tool::DeviceBuffer<Vector12> EE_d_grad;

        cuda_tool::DeviceBuffer<Vector4i> EEs_friction;
        cuda_tool::DeviceBuffer<Float>    EE_lambda_friction;

        cuda_tool::DeviceBuffer<int64_t>  ij_hash_input;
        cuda_tool::DeviceBuffer<int64_t>  ij_hash;
        cuda_tool::DeviceBuffer<int>      sort_index_input;
        cuda_tool::DeviceBuffer<int>      sort_index;
        cuda_tool::DeviceBuffer<int>      offset, unique_flag;
        cuda_tool::DeviceVar<int>         total_count;
        cuda_tool::DeviceBuffer<Vector2i> tmp_idx;
        cuda_tool::DeviceBuffer<Float>    tmp_lambda;
        cuda_tool::DeviceBuffer<int>      tmp_cnt;

        cuda_tool::DeviceBuffer<Vector3> non_penetrate_positions;

        Float                                   decay_factor;
        S<const geometry::AttributeSlot<Float>> dt_attr;
        Float                                   toi_threshold;
        Float                                   alpha_lower_bound;
        bool                                    energy_enabled;
        bool should_discard_friction_candidates = false;

        Float m_reserve_ratio = 1.5;

        template <typename U>
        void loose_resize(cuda_tool::DeviceBuffer<U>& buf, size_t new_size)
        {
            if(buf.capacity() < new_size)
                buf.reserve(new_size * m_reserve_ratio);
            buf.resize(new_size);
        }

        void init_mu();
        void filter_active();
        void update_active_set();
        void linearize_constraints();
        void update_slack();
        void update_lambda();
        void update_friction();
        void clear_friction_candidates();
        void snapshot_friction_candidates();

        void record_non_penetrate_positions();
        void recover_non_penetrate_positions();
        void advance_non_penetrate_positions(Float alpha);
        void prepare_ccd();
        void post_ccd();
    };

    cuda_tool::CBufferView<int>      PHs() const;
    cuda_tool::CBufferView<Float>    PH_d0() const;
    cuda_tool::CBufferView<Vector3>  PH_d_grad() const;
    cuda_tool::CBufferView<Float>    PH_lambda() const;
    cuda_tool::CBufferView<int>      PH_cnt() const;
    cuda_tool::CBufferView<Vector2i> PHs_friction() const;
    cuda_tool::CBufferView<Float>    PH_lambda_friction() const;

    cuda_tool::CBufferView<Vector4i> PTs() const;
    cuda_tool::CBufferView<Float>    PT_d0() const;
    cuda_tool::CBufferView<Vector12> PT_d_grad() const;
    cuda_tool::CBufferView<Float>    PT_lambda() const;
    cuda_tool::CBufferView<int>      PT_cnt() const;
    cuda_tool::CBufferView<Vector4i> PTs_friction() const;
    cuda_tool::CBufferView<Float>    PT_lambda_friction() const;

    cuda_tool::CBufferView<Vector4i> EEs() const;
    cuda_tool::CBufferView<Float>    EE_d0() const;
    cuda_tool::CBufferView<Vector12> EE_d_grad() const;
    cuda_tool::CBufferView<Float>    EE_lambda() const;
    cuda_tool::CBufferView<int>      EE_cnt() const;
    cuda_tool::CBufferView<Vector4i> EEs_friction() const;
    cuda_tool::CBufferView<Float>    EE_lambda_friction() const;

    cuda_tool::CBufferView<Vector3> non_penetrate_positions() const;

    cuda_tool::CBufferView<Float> mu_vertices() const;
    //tex: $\Gamma$
    Float decay_factor() const;
    Float toi_threshold() const;
    Float alpha_lower_bound() const;
    bool  is_enabled() const;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    friend class GlobalVertexManager;
    void init();

    void init_mu();
    void filter_active();
    void update_active_set();
    void linearize_constraints();
    void update_slack();
    void update_lambda();
    void update_friction();
    void clear_friction_candidates();
    void snapshot_friction_candidates();
    void require_discard_friction();
    void record_non_penetrate_positions();
    void recover_non_penetrate_positions();
    void advance_non_penetrate_positions(Float alpha);
    void prepare_ccd();
    void post_ccd();

    void enable();
    void disable();

    friend class ActiveSetReporter;
    void add_reporter(ActiveSetReporter* reporter);
    friend class ALStiffnessEstimator;
    void add_stiffness_estimator(ALStiffnessEstimator* estimator);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
