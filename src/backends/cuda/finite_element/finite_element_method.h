#pragma once
#include <sim_system.h>
#include <muda/buffer.h>
#include <uipc/geometry/simplicial_complex.h>
#include <global_geometry/global_vertex_manager.h>
#include <backends/cuda/utils/dump_utils.h>

namespace uipc::backend::cuda
{
class FiniteElementVertexReporter;
class FiniteElementSurfaceReporter;
class FiniteElementBodyReporter;

class FiniteElementKinetic;
class FiniteElementElastics;
class FiniteElementConstitution;
class FiniteElementExtraConstitution;

class FEM3DConstitution;
class Codim2DConstitution;
class Codim1DConstitution;
class Codim0DConstitution;

class FiniteElementDiffParmReporter;
class FiniteElementConstitutionDiffParmReporter;
class FiniteElementExtraConstitutionDiffParmReporter;

class FEM3DConstitutionDiffParmReporter;
class Codim2DConstitutionDiffParmReporter;
class Codim1DConstitutionDiffParmReporter;
class Codim0DConstitutionDiffParmReporter;

class FiniteElementDiffDofReporter;

class FiniteElementMethod final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class DimUID
    {
      public:
        IndexT dim = -1;
        U64    uid = ~0ull;
    };

    class GeoInfo
    {
      public:
        IndexT geo_slot_index = -1;  // slot index in the scene

        DimUID dim_uid;

        SizeT vertex_offset = ~0ull;
        SizeT vertex_count  = 0ull;

        SizeT primitive_offset = ~0ull;
        SizeT primitive_count  = 0ull;
    };

    class ConstitutionInfo
    {
      public:
        SizeT geo_info_offset = ~0ull;
        SizeT geo_info_count  = 0ull;

        SizeT vertex_offset = ~0ull;
        SizeT vertex_count  = 0ull;

        SizeT primitive_offset = ~0ull;
        SizeT primitive_count  = 0ull;
    };

    class DimInfo
    {
      public:
        SizeT geo_info_offset  = ~0ull;
        SizeT geo_info_count   = 0ull;
        SizeT primitive_offset = ~0ull;
        SizeT primitive_count  = 0ull;
        SizeT vertex_offset    = ~0ull;
        SizeT vertex_count     = 0ull;
    };

    class ForEachInfo
    {
      public:
        SizeT          global_index() const noexcept { return m_global_index; }
        SizeT          local_index() const noexcept { return m_local_index; }
        const GeoInfo& geo_info() const noexcept { return *m_geo_info; }

      private:
        friend class FiniteElementMethod;
        SizeT          m_global_index = 0;
        SizeT          m_local_index  = 0;
        const GeoInfo* m_geo_info     = nullptr;
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, IndexT dim, SizeT index_in_dim) noexcept
            : m_impl(impl)
            , m_dim(dim)
            , m_index_in_dim(index_in_dim)
        {
        }

        span<const GeoInfo> geo_infos() const noexcept;

        const ConstitutionInfo& constitution_info() const noexcept;

        size_t vertex_count() const noexcept;

        size_t primitive_count() const noexcept;

        /**
         * @brief For each primitive or vertex in the filtered info
         * 
         * @code
         *  vector<Float> lambdas(info.vertex_count());
         *  
         *  info.for_each(geo_slots, 
         *  [](SimplicialComplex& sc) 
         *  {
         *      return sc.vertices().find<Float>("lambda").view();
         *  },
         *  [&](const ForEachInfo& I, Float lambda)
         *  {
         *      lambdas[I.global_index()] = lambda;
         *  });
         * @endcode
         */
        template <typename ForEach, typename ViewGetter>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetter&&                    view_getter,
                      ForEach&&                       for_each_action) const;

        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ForEachGeometry&&               for_each) const;

      protected:
        friend class FiniteElementMethod;
        Impl*  m_impl         = nullptr;
        SizeT  m_index_in_dim = ~0ull;
        IndexT m_dim          = -1;
    };

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _init_dof_info();
        void _classify_base_constitutions();

        void _build_geo_infos(WorldVisitor& world);
        void _build_base_constitution_infos();
        void _build_on_host(WorldVisitor& world);
        void _build_on_device();
        void _download_geometry_to_host();
        void _init_base_constitutions();
        void _init_extra_constitutions();
        void _init_diff_reporters();

        void write_scene(WorldVisitor& world);
        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        // Forward Simulation:

        SimSystemSlot<GlobalVertexManager> global_vertex_manager;

        SimSystemSlot<FiniteElementElastics>               elastics;
        SimSystemSlotCollection<FiniteElementConstitution> constitutions;
        SimSystemSlotCollection<FiniteElementExtraConstitution> extra_constitutions;

        // Differentiable Simulation Systems:

        SimSystemSlot<FiniteElementDiffParmReporter> kinetic_diff_parm_reporter;
        SimSystemSlotCollection<FiniteElementConstitutionDiffParmReporter> constitution_diff_parm_reporters;
        SimSystemSlotCollection<FiniteElementExtraConstitutionDiffParmReporter> extra_constitution_diff_parm_reporters;


        // Core Invariant Data:

        vector<GeoInfo> geo_infos;


        // Related Data:

        std::array<DimInfo, 4> dim_infos;

        unordered_map<U64, SizeT>    codim_0d_uid_to_index;
        vector<Codim0DConstitution*> codim_0d_constitutions;
        vector<ConstitutionInfo>     codim_0d_constitution_infos;

        unordered_map<U64, SizeT>    codim_1d_uid_to_index;
        vector<Codim1DConstitution*> codim_1d_constitutions;
        vector<ConstitutionInfo>     codim_1d_constitution_infos;

        unordered_map<U64, SizeT>    codim_2d_uid_to_index;
        vector<Codim2DConstitution*> codim_2d_constitutions;
        vector<ConstitutionInfo>     codim_2d_constitution_infos;

        unordered_map<U64, SizeT>  fem_3d_uid_to_index;
        vector<FEM3DConstitution*> fem_3d_constitutions;
        vector<ConstitutionInfo>   fem_3d_constitution_infos;

        unordered_map<U64, SizeT> extra_constitution_uid_to_index;


        // Simulation Data:

        vector<IndexT> h_vertex_contact_element_ids;
        vector<IndexT> h_vertex_subscene_contact_element_ids;
        vector<Float>  h_vertex_d_hat;

        vector<IndexT>  h_vertex_is_fixed;
        vector<IndexT>  h_vertex_is_dynamic;
        vector<IndexT>  h_vertex_body_id;
        vector<Vector3> h_gravities;

        vector<Vector3> h_positions;
        vector<Vector3> h_rest_positions;
        vector<Vector3> h_velocities;
        vector<Float>   h_thicknesses;
        vector<IndexT>  h_dimensions;
        vector<Float>   h_masses;

        vector<IndexT>   h_codim_0ds;
        vector<Vector2i> h_codim_1ds;
        vector<Vector3i> h_codim_2ds;
        vector<Vector4i> h_tets;

        vector<IndexT> h_body_self_collision;

        Vector3 default_gravity;
        Float   default_d_hat;


        // Element Attributes:

        muda::DeviceBuffer<IndexT> codim_0ds;

        muda::DeviceBuffer<Vector2i> codim_1ds;
        muda::DeviceBuffer<Float>    rest_lengths;

        muda::DeviceBuffer<Vector3i> codim_2ds;
        muda::DeviceBuffer<Float>    rest_areas;

        muda::DeviceBuffer<Vector4i> tets;
        muda::DeviceBuffer<Float>    rest_volumes;


        // Vertex Attributes:

        muda::DeviceBuffer<IndexT>  is_fixed;    // Vertex IsFixed
        muda::DeviceBuffer<IndexT>  is_dynamic;  // Vertex IsDynamic
        muda::DeviceBuffer<Vector3> gravities;   // Vertex Gravity

        muda::DeviceBuffer<Vector3> x_bars;    // Rest Positions
        muda::DeviceBuffer<Vector3> xs;        // Positions
        muda::DeviceBuffer<Vector3> dxs;       // Displacements
        muda::DeviceBuffer<Vector3> x_temps;   // Safe Positions for line search
        muda::DeviceBuffer<Vector3> vs;        // Velocities
        muda::DeviceBuffer<Vector3> x_tildes;  // Predicted Positions
        muda::DeviceBuffer<Vector3> x_prevs;   // Positions at last frame
        muda::DeviceBuffer<Float>   masses;    // Mass
        muda::DeviceBuffer<Float>   thicknesses;  // Thickness


        //tex:
        // FEM3D Material Basis
        //$$
        // \begin{bmatrix}
        // \bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_0 \\
        // \end{bmatrix}^{-1}
        // $$
        muda::DeviceBuffer<Matrix3x3> Dm3x3_invs;


        // Dump:

        BufferDump dump_xs;       // Positions
        BufferDump dump_x_prevs;  // Positions at last frame
        BufferDump dump_vs;       // Velocities

        // Dof Info:

        void set_dof_info(SizeT frame, IndexT dof_offset, IndexT dof_count);  // only called by FEMLinearSubsystem
        IndexT dof_offset(SizeT frame) const noexcept;
        IndexT dof_count(SizeT frame) const noexcept;

      private:
        vector<IndexT> frame_to_dof_offset;
        vector<IndexT> frame_to_dof_count;
    };


  public:
    auto codim_0ds() const noexcept { return m_impl.codim_0ds.view(); }
    auto codim_1ds() const noexcept { return m_impl.codim_1ds.view(); }
    auto codim_2ds() const noexcept { return m_impl.codim_2ds.view(); }
    auto tets() const noexcept { return m_impl.tets.view(); }

    auto is_fixed() const noexcept { return m_impl.is_fixed.view(); }
    auto is_dynamic() const noexcept { return m_impl.is_dynamic.view(); }
    auto x_bars() const noexcept { return m_impl.x_bars.view(); }
    auto xs() const noexcept { return m_impl.xs.view(); }
    auto dxs() const noexcept { return m_impl.dxs.view(); }
    auto x_temps() const noexcept { return m_impl.x_temps.view(); }
    auto vs() const noexcept { return m_impl.vs.view(); }
    auto x_tildes() const noexcept { return m_impl.x_tildes.view(); }
    auto x_prevs() const noexcept { return m_impl.x_prevs.view(); }
    auto masses() const noexcept { return m_impl.masses.view(); }
    auto thicknesses() const noexcept { return m_impl.thicknesses.view(); }
    auto rest_volumes() const noexcept { return m_impl.rest_volumes.view(); }
    auto rest_areas() const noexcept { return m_impl.rest_areas.view(); }
    auto rest_lengths() const noexcept { return m_impl.rest_lengths.view(); }
    auto Dm3x3_invs() const noexcept { return m_impl.Dm3x3_invs.view(); }
    auto gravities() const noexcept { return m_impl.gravities.view(); }

    /**
     * @brief return the frame-local dof offset of FEM for the given frame
     */
    IndexT dof_offset(SizeT frame) const noexcept;
    /**
     * @brief return the frame-local dof count of FEM for the given frame
     */
    IndexT dof_count(SizeT frame) const noexcept;

    /**
     * @brief For each primitive
     * 
     * @code
     *  vector<Float> lambdas(info.vertex_count());
     *  
     *  for_each(geo_slots, 
     *  [](SimplicialComplex& sc) 
     *  {
     *      return sc.vertices().find<Float>("lambda").view();
     *  },
     *  [&](const ForEachInfo& I, Float lambda)
     *  {
     *      auto vI = I.globl_index();
     *      lambdas[vI] = lambda;
     *  });
     * @endcode
     */
    template <typename ForEach, typename ViewGetter>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                  ViewGetter&&                    view_getter,
                  ForEach&&                       for_each_action);

    /**
     * @brief For each geometry
     *  
     * @code
     *  for_each(geo_slots,
     *  [](const ForEachInfo& I,SimplicialComplex& sc)
     *  {
     *      auto geoI = I.global_index();
     *      ...
     *  });
     * @endcode
     */
    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_each);

  private:
    friend class FiniteElementVertexReporter;
    friend class FiniteElementSurfaceReporter;
    friend class FiniteElementBodyReporter;

    friend class FEMLinearSubsystem;
    friend class FEMLineSearchReporter;


    friend class FiniteElementAnimator;
    friend class FEMDiagPreconditioner;

    friend class FiniteElementDiffDofReporter;
    friend class FiniteElementKineticDiffParmReporter;
    friend class FEMAdjointMethodReplayer;
    friend class FEMTimeIntegrator;

    friend class FiniteElementStateAccessorFeatureOverrider;

    friend class SimEngine;
    void init();  // only be called by SimEngine

    friend class FiniteElementElastics;
    void add_elastics(FiniteElementElastics* elastics);
    friend class FiniteElementConstitution;
    void add_constitution(FiniteElementConstitution* constitution);  // only called by FiniteElementConstitution
    friend class FiniteElementExtraConstitution;
    void add_constitution(FiniteElementExtraConstitution* constitution);  // only called by FiniteElementExtraConstitution


    friend class FiniteElementConstitutionDiffParmReporter;
    void add_reporter(FiniteElementConstitutionDiffParmReporter* reporter);  // only called by FiniteElementConstitutionDiffParmReporter
    friend class FiniteElementExtraConstitutionDiffParmReporter;
    void add_reporter(FiniteElementExtraConstitutionDiffParmReporter* reporter);  // only called by FiniteElementExtraConstitutionDiffParmReporter

    friend class FiniteElementDiffParmReporter;
    void add_kinetic_reporter(FiniteElementDiffParmReporter* reporter);  // only called by FiniteElementKineticDiffParmReporter


    // Internal:

    template <typename ForEach, typename ViewGetter>
    static void _for_each(span<const GeoInfo>             geo_infos,
                          span<S<geometry::GeometrySlot>> geo_slots,
                          ViewGetter&&                    view_getter,
                          ForEach&&                       for_each_action);

    template <typename ForEachGeometry>
    static void _for_each(span<const GeoInfo>             geo_infos,
                          span<S<geometry::GeometrySlot>> geo_slots,
                          ForEachGeometry&&               for_each);

    virtual void do_build() override;

    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/finite_element_method.inl"