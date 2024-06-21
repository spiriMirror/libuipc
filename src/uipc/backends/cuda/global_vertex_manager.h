#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <functional>
#include <Eigen/Geometry>
#include <collision_detection/aabb.h>

namespace uipc::backend::cuda
{
class GlobalVertexManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class VertexCountInfo
    {
      public:
        void  count(SizeT count) noexcept;
        SizeT count() const noexcept;

      private:
        SizeT m_count;
    };

    class VertexAttributes
    {
      public:
        VertexAttributes() = default;
        muda::BufferView<IndexT>  coindex() const noexcept;
        muda::BufferView<Vector3> positions() const noexcept;

      private:
        friend class GlobalVertexManager;

        muda::BufferView<IndexT>  m_coindex;
        muda::BufferView<Vector3> m_positions;
    };

    class VertexDisplacement
    {
      public:
        VertexDisplacement() = default;
        muda::BufferView<Vector3> displacements() const noexcept;

      private:
        friend class GlobalVertexManager;

        muda::BufferView<Vector3> m_safe_positions;
        muda::BufferView<Vector3> m_displacements;
    };

    void on_update(std::function<void(VertexCountInfo&)>&& report_vertex_count,
                   std::function<void(VertexAttributes&)>&& report_vertex_attributes,
                   std::function<void(VertexDisplacement&)>&& report_vertex_displacement);

    muda::CBufferView<IndexT>  coindex() const noexcept;
    muda::CBufferView<Vector3> positions() const noexcept;
    muda::CBufferView<Vector3> displacements() const noexcept;

    Float compute_max_displacement();

    AABB compute_vertex_bounding_box();

  public:
    class Impl
    {
        friend class GlobalVertexManager;

      public:
        Impl() = default;
        void build_vertex_info();
        void on_update(std::function<void(VertexCountInfo&)>&& report_vertex_count,
                       std::function<void(VertexAttributes&)>&& report_vertex_attributes,
                       std::function<void(VertexDisplacement&)>&& report_vertex_displacement);
        Float compute_max_displacement();
        AABB  compute_vertex_bounding_box();

      private:
        /**
         * @brief A mapping from the global vertex index to the coindex.
         * 
         * The values of coindex is dependent on the reporters, which can be:
         * 1) the local index of the vertex
         * 2) or any other information that is needed to be stored.
         */
        muda::DeviceBuffer<IndexT>  coindex;
        muda::DeviceBuffer<Vector3> positions;
        muda::DeviceBuffer<Vector3> displacements;

        muda::DeviceVar<Float>   max_disp;
        muda::DeviceVar<Vector3> min_pos;
        muda::DeviceVar<Vector3> max_pos;

        list<std::function<void(VertexCountInfo&)>>  vertex_count_reporter;
        list<std::function<void(VertexAttributes&)>> vertex_attribute_reporter;
        list<std::function<void(VertexDisplacement&)>> vertex_displacement_reporter;
    };

  private:
    friend class SimEngine;
    void build_vertex_info();  // only be called by SimEngine

    Impl m_impl;
};
}  // namespace uipc::backend::cuda