#pragma once
#include <collision_detection/trajectory_filter.h>
#include <collision_detection/global_trajectory_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/global_body_manager.h>
#include <contact_system/global_contact_manager.h>
#include <muda/buffer/device_buffer.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class SimplexTrajectoryFilter : public TrajectoryFilter
{
  public:
    using TrajectoryFilter::TrajectoryFilter;

    class Impl;

    class BuildInfo
    {
      public:
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        Float d_hat() const noexcept;

        // Vertex Attributes

        /**
         * @brief Vertex Id to Body Id mapping.
         */
        muda::CBufferView<Float>    d_hats() const noexcept;
        muda::CBufferView<IndexT>   v2b() const noexcept;
        muda::CBufferView<Vector3>  positions() const noexcept;
        muda::CBufferView<Vector3>  rest_positions() const noexcept;
        muda::CBufferView<Float>    thicknesses() const noexcept;
        muda::CBufferView<IndexT>   dimensions() const noexcept;
        muda::CBufferView<IndexT>   contact_element_ids() const noexcept;
        muda::CBufferView<IndexT>   subscene_element_ids() const noexcept;
        muda::CBuffer2DView<IndexT> contact_mask_tabular() const noexcept;
        muda::CBuffer2DView<IndexT> subscene_mask_tabular() const noexcept;
        // Body Attributes

        /**
         * @brief Tell if the body needs self-collision
         */
        muda::CBufferView<IndexT> body_self_collision() const noexcept;

        // Topologies

        muda::CBufferView<IndexT>   codim_vertices() const noexcept;
        muda::CBufferView<IndexT>   surf_vertices() const noexcept;
        muda::CBufferView<Vector2i> surf_edges() const noexcept;
        muda::CBufferView<Vector3i> surf_triangles() const noexcept;

      protected:
        friend class SimplexTrajectoryFilter;
        Impl* m_impl = nullptr;
    };

    class DetectInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        Float alpha() const noexcept { return m_alpha; }

        muda::CBufferView<Vector3> displacements() const noexcept;

      private:
        friend class SimplexTrajectoryFilter;
        Float m_alpha = 0.0;
    };

    class FilterActiveInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        /**
         * @brief Candidate point-triangle pairs.
         */
        void PTs(muda::CBufferView<Vector4i> PTs) noexcept;
        /**
         * @brief Candidate edge-edge pairs.
         */
        void EEs(muda::CBufferView<Vector4i> EEs) noexcept;
        /**
         * @brief Candidate point-edge pairs.
         */
        void PEs(muda::CBufferView<Vector3i> PEs) noexcept;
        /**
         * @brief Candidate point-point pairs.
         */
        void PPs(muda::CBufferView<Vector2i> PPs) noexcept;
    };

    class FilterTOIInfo : public DetectInfo
    {
      public:
        using DetectInfo::DetectInfo;

        muda::VarView<Float> toi() noexcept;

      private:
        friend class SimplexTrajectoryFilter;
        muda::VarView<Float> m_toi;
    };

    class Impl
    {
      public:
        void record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info);
        void label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info);
        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        SimSystemSlot<GlobalVertexManager> global_vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager> global_simplicial_surface_manager;
        SimSystemSlot<GlobalContactManager> global_contact_manager;
        SimSystemSlot<GlobalBodyManager>    global_body_manager;

        muda::CBufferView<Vector4i> PTs;
        muda::CBufferView<Vector4i> EEs;
        muda::CBufferView<Vector3i> PEs;
        muda::CBufferView<Vector2i> PPs;

        muda::DeviceBuffer<Vector4i> friction_PT;
        muda::DeviceBuffer<Vector4i> friction_EE;
        muda::DeviceBuffer<Vector3i> friction_PE;
        muda::DeviceBuffer<Vector2i> friction_PP;

        muda::DeviceBuffer<Vector4i> recovered_PT;
        muda::DeviceBuffer<Vector4i> recovered_EE;
        muda::DeviceBuffer<Vector3i> recovered_PE;
        muda::DeviceBuffer<Vector2i> recovered_PP;

        Float reserve_ratio = 1.1;

        BufferDump dump_PTs;
        BufferDump dump_EEs;
        BufferDump dump_PEs;
        BufferDump dump_PPs;

        template <typename T>
        void loose_resize(muda::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve(size * reserve_ratio);
            }
            buffer.resize(size);
        }
    };

    muda::CBufferView<Vector4i> PTs() const noexcept;
    muda::CBufferView<Vector4i> EEs() const noexcept;
    muda::CBufferView<Vector3i> PEs() const noexcept;
    muda::CBufferView<Vector2i> PPs() const noexcept;

    muda::CBufferView<Vector4i> friction_PTs() const noexcept;
    muda::CBufferView<Vector4i> friction_EEs() const noexcept;
    muda::CBufferView<Vector3i> friction_PEs() const noexcept;
    muda::CBufferView<Vector2i> friction_PPs() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                = 0;
    virtual void do_detect(DetectInfo& info)              = 0;
    virtual void do_filter_active(FilterActiveInfo& info) = 0;
    virtual void do_filter_toi(FilterTOIInfo& info)       = 0;
    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

  private:
    friend class GlobalDCDFilter;
    Impl m_impl;

    virtual void do_build() override final;

    virtual void do_detect(GlobalTrajectoryFilter::DetectInfo& info) override final;
    virtual void do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual void do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info) override final;
    virtual void do_record_friction_candidates(
        GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info) override final;
    virtual void do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info) final override;
    virtual void do_clear_friction_candidates() override final;
};
}  // namespace uipc::backend::cuda
