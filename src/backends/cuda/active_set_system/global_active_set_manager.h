#pragma once

#include <sim_system.h>
#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>

namespace uipc::backend::cuda
{

class ActiveSetReporter;

class GlobalActiveSetManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class NonPenetratePositionInfo
    {
      public:
        NonPenetratePositionInfo(Impl* impl, SizeT offset, SizeT count) noexcept;
        muda::BufferView<Vector3> non_penetrate_positions() const noexcept;

      private:
        friend class GlobalActiveSetManager;
        Impl* m_impl;
        SizeT m_offset;
        SizeT m_count;
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

        SimSystemSlotCollection<ActiveSetReporter> active_set_reporters;

        muda::DeviceBuffer<Vector2i> PH_idx;
        muda::DeviceBuffer<Float> PH_lambda;
        muda::DeviceBuffer<int> PH_cnt;

        muda::DeviceBuffer<int> PHs;
        muda::DeviceBuffer<Float> PH_d0, PH_slack;
        muda::DeviceBuffer<Vector3> PH_d_grad;

        muda::DeviceBuffer<Vector2i> PT_idx;
        muda::DeviceBuffer<Float>    PT_lambda;
        muda::DeviceBuffer<int>      PT_cnt;

        muda::DeviceBuffer<Vector4i> PTs;
        muda::DeviceBuffer<Float>    PT_d0, PT_slack;
        muda::DeviceBuffer<Vector12> PT_d_grad;

        muda::DeviceBuffer<Vector2i> EE_idx;
        muda::DeviceBuffer<Float>    EE_lambda;
        muda::DeviceBuffer<int>      EE_cnt;

        muda::DeviceBuffer<Vector4i> EEs;
        muda::DeviceBuffer<Float>    EE_d0, EE_slack;
        muda::DeviceBuffer<Vector12> EE_d_grad;

        muda::DeviceBuffer<int64_t>  ij_hash_input;
        muda::DeviceBuffer<int64_t>  ij_hash;
        muda::DeviceBuffer<int>      sort_index_input;
        muda::DeviceBuffer<int>      sort_index;
        muda::DeviceBuffer<int>      offset, unique_flag;
        muda::DeviceVar<int>         total_count;
        muda::DeviceBuffer<Vector2i> tmp_idx;
        muda::DeviceBuffer<Float>    tmp_lambda;
        muda::DeviceBuffer<int>      tmp_cnt;

        muda::DeviceBuffer<Vector3> non_penetrate_positions;

        Float mu, decay_factor, mu_scale;
        Float toi_threshold;
        bool  energy_enabled;

        Float m_reserve_ratio = 1.5;

        template <typename U>
        void loose_resize(muda::DeviceBuffer<U>& buf, size_t new_size)
        {
            if(buf.capacity() < new_size)
                buf.reserve(new_size * m_reserve_ratio);
            buf.resize(new_size);
        }

        void filter_active();
        void update_active_set();
        void linearize_constraints();
        void update_slack();
        void update_lambda();

        void record_non_penetrate_positions();
        void recover_non_penetrate_positions();
        void advance_non_penetrate_positions(Float alpha);
    };

    muda::CBufferView<int>            PHs() const;
    muda::CBufferView<Float>          PH_d0() const;
    muda::CBufferView<Vector3>        PH_d_grad() const;
    muda::CBufferView<Float>          PH_lambda() const;
    muda::CBufferView<int>            PH_cnt() const;

    muda::CBufferView<Vector4i>       PTs() const;
    muda::CBufferView<Float>          PT_d0() const;
    muda::CBufferView<Vector12>       PT_d_grad() const;
    muda::CBufferView<Float>          PT_lambda() const;
    muda::CBufferView<int>            PT_cnt() const;

    muda::CBufferView<Vector4i> EEs() const;
    muda::CBufferView<Float>    EE_d0() const;
    muda::CBufferView<Vector12> EE_d_grad() const;
    muda::CBufferView<Float>    EE_lambda() const;
    muda::CBufferView<int>      EE_cnt() const;

    muda::CBufferView<Vector3> non_penetrate_positions() const;

    Float mu() const;
    Float mu_scale() const;
    //tex: $\Gamma$
    Float decay_factor() const;
    Float toi_threshold() const;
    bool  is_enabled() const;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    void init();

    void filter_active();
    void update_active_set();
    void linearize_constraints();
    void update_slack();
    void update_lambda();
    void record_non_penetrate_positions();
    void recover_non_penetrate_positions();
    void advance_non_penetrate_positions(Float alpha);

    void enable();
    void disable();
    void mu(Float mu);

    friend class ActiveSetReporter;
    void add_reporter(ActiveSetReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
