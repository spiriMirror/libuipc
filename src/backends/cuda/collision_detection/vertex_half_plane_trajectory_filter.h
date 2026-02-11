#pragma once
#include <collision_detection/trajectory_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <muda/buffer/device_buffer.h>
#include <implicit_geometry/half_plane.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class HalfPlaneVertexReporter;
class VertexHalfPlaneTrajectoryFilter : public TrajectoryFilter
{
  public:
    using TrajectoryFilter::TrajectoryFilter;

    class Impl;

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        Float                    d_hat() const noexcept;
        muda::CBufferView<Float> d_hats() const noexcept;

        IndexT                     half_plane_vertex_offset() const noexcept;
        muda::CBufferView<Vector3> plane_normals() const noexcept;
        muda::CBufferView<Vector3> plane_positions() const noexcept;

        muda::CBufferView<Vector3>  positions() const noexcept;
        muda::CBufferView<Float>    thicknesses() const noexcept;
        muda::CBufferView<IndexT>   contact_element_ids() const noexcept;
        muda::CBufferView<IndexT>   subscene_element_ids() const noexcept;
        muda::CBuffer2DView<IndexT> contact_mask_tabular() const noexcept;
        muda::CBuffer2DView<IndexT> subscene_mask_tabular() const noexcept;
        muda::CBufferView<IndexT>   surf_vertices() const noexcept;

      private:
        friend class VertexHalfPlaneTrajectoryFilter;
        Impl* m_impl;
    };

    class DetectInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        Float                      alpha() const noexcept;
        muda::CBufferView<Vector3> displacements() const noexcept;

      private:
        friend class VertexHalfPlaneTrajectoryFilter;
        Float m_alpha;
    };

    class FilterActiveInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        /**
         * @brief Candidate vertex-half-plane pairs.
         */
        void PHs(muda::CBufferView<Vector2i> Ps) noexcept;
    };

    class FilterTOIInfo : public DetectInfo
    {
      public:
        using DetectInfo::DetectInfo;

        muda::VarView<Float> toi() noexcept;

      private:
        friend class VertexHalfPlaneTrajectoryFilter;
        muda::VarView<Float> m_toi;
    };

    class BuildInfo
    {
      public:
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

        GlobalVertexManager* global_vertex_manager = nullptr;
        GlobalSimplicialSurfaceManager* global_simplicial_surface_manager = nullptr;
        GlobalContactManager*    global_contact_manager     = nullptr;
        HalfPlane*               half_plane                 = nullptr;
        HalfPlaneVertexReporter* half_plane_vertex_reporter = nullptr;

        muda::CBufferView<Vector2i>  PHs;
        muda::DeviceBuffer<Vector2i> friction_PHs;
        muda::DeviceBuffer<Vector2i> recovered_PHs;

        Float reserve_ratio = 1.1;

        BufferDump dump_PHs;

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

    muda::CBufferView<Vector2i> PHs() noexcept;
    muda::CBufferView<Vector2i> friction_PHs() noexcept;

  protected:
    virtual void do_detect(DetectInfo& info)              = 0;
    virtual void do_filter_active(FilterActiveInfo& info) = 0;
    virtual void do_filter_toi(FilterTOIInfo& info)       = 0;

    virtual void do_build(BuildInfo& info){};
    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

  private:
    Impl         m_impl;
    virtual void do_build() override final;

    virtual void do_detect(GlobalTrajectoryFilter::DetectInfo& info) override final;
    virtual void do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual void do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info) override final;
    virtual void do_record_friction_candidates(
        GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info) override final;
    virtual void do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info) override final;
    virtual void do_clear_friction_candidates() override final;
};
}  // namespace uipc::backend::cuda
