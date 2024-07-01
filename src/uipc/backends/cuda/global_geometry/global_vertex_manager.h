#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <functional>
#include <Eigen/Geometry>
#include <collision_detection/aabb.h>

namespace uipc::backend::cuda
{
class VertexReporter;
class GlobalVertexManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class VertexCountInfo
    {
      public:
        void count(SizeT count) noexcept;
        void changable(bool is_changable) noexcept;

      private:
        friend class GlobalVertexManager;
        SizeT m_count;
        bool  m_changable;
    };

    class VertexAttributeInfo
    {
      public:
        VertexAttributeInfo(Impl* impl, SizeT index) noexcept;
        muda::BufferView<IndexT>  coindices() const noexcept;
        muda::BufferView<Vector3> positions() const noexcept;
        muda::BufferView<IndexT>  contact_element_ids() const noexcept;

      private:
        friend class GlobalVertexManager;
        SizeT m_index;
        Impl* m_impl;
    };

    class VertexDisplacementInfo
    {
      public:
        VertexDisplacementInfo(Impl* impl, SizeT index) noexcept;
        muda::BufferView<Vector3> displacements() const noexcept;
        muda::CBufferView<IndexT> coindices() const noexcept;

      private:
        friend class GlobalVertexManager;
        SizeT m_index;
        Impl* m_impl;
    };

    class Impl;

    muda::CBufferView<IndexT>  coindices() const noexcept;
    muda::CBufferView<Vector3> positions() const noexcept;
    muda::CBufferView<Vector3> safe_positions() const noexcept;
    muda::CBufferView<IndexT>  contact_element_ids() const noexcept;
    muda::CBufferView<Vector3> displacements() const noexcept;

    void add_reporter(VertexReporter* reporter);

    AABB vertex_bounding_box() const noexcept;

  public:
    class Impl
    {
        friend class GlobalVertexManager;

      public:
        Impl() = default;
        void  init_vertex_info();
        void  rebuild_vertex_info();
        void  step_forward(Float alpha);
        Float compute_max_displacement();
        AABB  compute_vertex_bounding_box();
        void  collect_vertex_displacements();
        void  record_start_point();

        template <typename T>
        muda::BufferView<T> subview(muda::DeviceBuffer<T>& buffer, SizeT index) const noexcept;

      private:
        /**
         * @brief A mapping from the global vertex index to the coindices.
         * 
         * The values of coindices is dependent on the reporters, which can be:
         * 1) the local index of the vertex
         * 2) or any other information that is needed to be stored.
         */
        muda::DeviceBuffer<IndexT>  coindices;
        muda::DeviceBuffer<Vector3> positions;
        muda::DeviceBuffer<Vector3> safe_positions;
        muda::DeviceBuffer<IndexT>  contact_element_ids;
        muda::DeviceBuffer<Vector3> displacements;

        muda::DeviceVar<Float>   max_disp;
        muda::DeviceVar<Vector3> min_pos;
        muda::DeviceVar<Vector3> max_pos;

        list<VertexReporter*>   vertex_reporter_buffer;
        vector<VertexReporter*> vertex_reporters;
        vector<VertexReporter*> changable_vertex_reporters;

        vector<SizeT> reporter_vertex_offsets;
        vector<SizeT> reporter_vertex_counts;

        AABB vertex_bounding_box;
    };

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    void  init_vertex_info();
    void  rebuild_vertex_info();
    Float compute_max_displacement();
    AABB  compute_vertex_bounding_box();
    void  step_forward(Float alpha);
    void  record_start_point();
    Impl  m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/global_vertex_manager.inl"
