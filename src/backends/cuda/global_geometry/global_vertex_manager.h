#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <functional>
#include <Eigen/Geometry>
#include <collision_detection/aabb.h>
#include <utils/dump_utils.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class GlobalTrajectoryFilter;
class VertexReporter;
class GlobalVertexManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class VertexCountInfo
    {
      public:
        void count(SizeT count) noexcept;
        void changeable(bool is_changable) noexcept;

      private:
        friend class GlobalVertexManager;
        friend class VertexReporter;

        SizeT m_count     = 0;
        bool  m_changable = false;
    };

    class VertexAttributeInfo
    {
      public:
        VertexAttributeInfo(Impl* impl, SizeT index, SizeT frame) noexcept;
        SizeT                     frame() const noexcept;
        muda::BufferView<Vector3> rest_positions() const noexcept;
        muda::BufferView<Float>   thicknesses() const noexcept;
        muda::BufferView<IndexT>  coindices() const noexcept;
        muda::BufferView<IndexT>  dimensions() const noexcept;
        muda::BufferView<Vector3> positions() const noexcept;
        muda::BufferView<IndexT>  contact_element_ids() const noexcept;
        muda::BufferView<IndexT>  subscene_element_ids() const noexcept;
        muda::BufferView<IndexT>  body_ids() const noexcept;
        // vert-wise d_hat
        muda::BufferView<Float> d_hats() const noexcept;
        /**
         * @breif require discard friction, if this update will ruin the friction computation
         * 
         */
        void require_discard_friction() const noexcept;

      private:
        friend class GlobalVertexManager;
        SizeT m_index;
        Impl* m_impl;
        SizeT m_frame;
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

    /**
     * @brief A mapping from the global vertex index to the coindices.
     * 
     * The values of coindices is dependent on the reporters, which can be:
     * 1) the local index of the vertex
     * 2) or any other information that is needed to be stored.
     */
    muda::CBufferView<IndexT> coindices() const noexcept;

    /**
     * @brief A mapping from the global vertex index to the body id.
     */
    muda::CBufferView<IndexT> body_ids() const noexcept;

    /**
     * @brief The d_hat of the vertices.
     * 
     * The d_hat is used to compute the penetration depth.
     */
    muda::CBufferView<Float> d_hats() const noexcept;

    /**
     * @brief The current positions of the vertices.
     */
    muda::CBufferView<Vector3> positions() const noexcept;

    /**
     * @brief The positions of the vertices at last time step.
     * 
     * Used to compute the friction.
     */
    muda::CBufferView<Vector3> prev_positions() const noexcept;

    /**
     * @brief The rest positions of the vertices.
     * 
     * Can be used to retrieve some quantities at the rest state.
     */
    muda::CBufferView<Vector3> rest_positions() const noexcept;

    /**
     * @brief The safe positions of the vertices in line search.
     *  
     * Used as a start point to do the line search.
     */
    muda::CBufferView<Vector3> safe_positions() const noexcept;

    /**
     * @brief Indicate the contact element id of the vertices.
     */
    muda::CBufferView<IndexT> contact_element_ids() const noexcept;

    /**
     * @brief Indicate the contact element id of the vertices.
     */
    muda::CBufferView<IndexT> subscene_element_ids() const noexcept;

    /**
     * @brief The displacements of the vertices (after solving the linear system).
     * 
     * The displacements are not scaled by the alpha.
     */
    muda::CBufferView<Vector3> displacements() const noexcept;

    /**
     * @brief The thicknesses of the vertices.
     * 
     * The thicknesses are used to compute the penetration depth.
     */
    muda::CBufferView<Float> thicknesses() const noexcept;

    /**
     * @brief The dimension of the vertices. 
     * - 0: Codim 0D
     * - 1: Codim 1D
     * - 2: Codim 2D
     * - 3: 3D
     */
    muda::CBufferView<IndexT> dimensions() const noexcept;

    /**
     * @brief the axis align bounding box of the all vertices.
     */
    AABB vertex_bounding_box() const noexcept;

  public:
    class Impl
    {
      public:
        Impl() = default;
        void init();
        void update_attributes(SizeT frame);
        void rebuild();

        void record_prev_positions();
        void record_start_point();
        void step_forward(Float alpha);

        void collect_vertex_displacements();

        Float compute_axis_max_displacement();
        AABB  compute_vertex_bounding_box();

        template <typename T>
        muda::BufferView<T> subview(muda::DeviceBuffer<T>& buffer, SizeT index) const noexcept;

        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        Float default_d_hat = 0.01;

        muda::DeviceBuffer<IndexT>  coindices;
        muda::DeviceBuffer<IndexT>  body_ids;
        muda::DeviceBuffer<Float>   d_hats;
        muda::DeviceBuffer<IndexT>  dimensions;
        muda::DeviceBuffer<Vector3> positions;
        muda::DeviceBuffer<Vector3> prev_positions;
        muda::DeviceBuffer<Vector3> rest_positions;
        muda::DeviceBuffer<Vector3> safe_positions;
        muda::DeviceBuffer<Float>   thicknesses;
        muda::DeviceBuffer<IndexT>  contact_element_ids;
        muda::DeviceBuffer<IndexT>  subscene_element_ids;
        muda::DeviceBuffer<Vector3> displacements;
        muda::DeviceBuffer<Float>   displacement_norms;

        muda::DeviceVar<Float>   axis_max_disp;
        muda::DeviceVar<Float>   max_disp_norm;
        muda::DeviceVar<Vector3> min_pos;
        muda::DeviceVar<Vector3> max_pos;


        SimSystemSlot<GlobalTrajectoryFilter>   global_trajectory_filter;
        SimSystemSlotCollection<VertexReporter> vertex_reporters;

        OffsetCountCollection<IndexT> reporter_vertex_offsets_counts;

        AABB vertex_bounding_box;

        BufferDump dump_positions;
        BufferDump dump_prev_positions;
    };

  protected:
    virtual void do_build() override;
    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

  private:
    friend class SimEngine;
    friend class MaxTranslationChecker;
    friend class GlobalTrajectoryFilter;

    // Initialize the global vertex manager
    // - Create the surface mesh
    // - Setup surface attributes
    void init();

    // Update the surface attributes
    void update_attributes();

    void  rebuild();
    void  record_prev_positions();
    void  collect_vertex_displacements();
    Float compute_axis_max_displacement();

    AABB compute_vertex_bounding_box();
    void step_forward(Float alpha);
    void record_start_point();

    friend class VertexReporter;
    void add_reporter(VertexReporter* reporter);
    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/global_vertex_manager.inl"
