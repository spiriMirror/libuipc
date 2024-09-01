#pragma once
#include <sim_system.h>
#include <muda/buffer.h>
#include <dof_predictor.h>
#include <uipc/geometry/simplicial_complex.h>
#include <global_geometry/global_vertex_manager.h>

namespace uipc::backend::cuda
{
class FiniteElementConstitution;
class FEM3DConstitution;
class Codim2DConstitution;
class Codim1DConstitution;
class Codim0DConstitution;

class FiniteElementMethod : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class DimUID
    {
      public:
        SizeT dim = ~0ull;
        U64   uid = ~0ull;
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
    };

    template <int N>
    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl) noexcept
            : m_impl(impl)
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
         *  
         *  vector<Float> lambdas(info.vertex_count());
         * 
         *  info.for_each(geo_slots, 
         *  [](SimplicialComplex& sc) 
         *  {
         *      return sc.vertices().find<Float>("lambda").view();
         *  },
         *  [&](SizeT I, Float lambda)
         *  {
         *      lambdas[I] = lambda;
         *  });
         * 
         * @endcode
         */
        template <typename ForEach, typename ViewGetter>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetter&&                    view_getter,
                      ForEach&& for_each_action) const noexcept;

      protected:
        friend class FiniteElementMethod;
        Impl* m_impl         = nullptr;
        SizeT m_index_in_dim = ~0ull;
    };

    using Codim0DFilteredInfo = FilteredInfo<0>;
    using Codim1DFilteredInfo = FilteredInfo<1>;
    using Codim2DFilteredInfo = FilteredInfo<2>;
    using FEM3DFilteredInfo   = FilteredInfo<3>;

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _init_constitutions();
        void _build_geo_infos(WorldVisitor& world);
        void _build_constitution_infos();

        void _build_on_host(WorldVisitor& world);
        void _build_on_device();
        void _download_geometry_to_host();
        void _distribute_constitution_filtered_info();

        void compute_x_tilde(DoFPredictor::PredictInfo& info);

        void compute_velocity(DoFPredictor::ComputeVelocityInfo& info);

        void write_scene(WorldVisitor& world);

        GlobalVertexManager* global_vertex_manager = nullptr;

        // core invariant data:
        vector<GeoInfo>                                    geo_infos;
        SimSystemSlotCollection<FiniteElementConstitution> constitutions;

        // related data:
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


        // simulation data:
        vector<IndexT>  h_vertex_contact_element_ids;
        vector<IndexT>  h_vertex_is_fixed;
        vector<Vector3> h_positions;
        vector<Vector3> h_rest_positions;
        vector<Float>   h_thicknesses;
        vector<Float>   h_masses;

        vector<IndexT>   h_codim_0ds;
        vector<Vector2i> h_codim_1ds;
        vector<Vector3i> h_codim_2ds;
        vector<Vector4i> h_tets;
        Vector3          gravity;

        // Element Attributes:

        muda::DeviceBuffer<IndexT> codim_0ds;

        muda::DeviceBuffer<Vector2i> codim_1ds;
        muda::DeviceBuffer<Float>    rest_lengths;

        muda::DeviceBuffer<Vector3i> codim_2ds;
        muda::DeviceBuffer<Float>    rest_areas;

        muda::DeviceBuffer<Vector4i> tets;
        muda::DeviceBuffer<Float>    rest_volumes;

        // Vertex Attributes:
        muda::DeviceBuffer<IndexT>  is_fixed;  // Vertex Fixed
        muda::DeviceBuffer<Vector3> x_bars;    // Rest Positions
        muda::DeviceBuffer<Vector3> xs;        // Positions
        muda::DeviceBuffer<Vector3> dxs;       // Displacements
        muda::DeviceBuffer<Vector3> x_temps;   // Safe Positions for line search
        muda::DeviceBuffer<Vector3> vs;        // Velocities
        muda::DeviceBuffer<Vector3> x_tildes;  // Predicted Positions
        muda::DeviceBuffer<Vector3> x_prevs;   // Positions at last frame
        muda::DeviceBuffer<Float>   masses;    // Mass
        muda::DeviceBuffer<Float>   thicknesses;      // Thickness
        muda::DeviceBuffer<Matrix3x3> diag_hessians;  // Diagonal Hessian

        //tex:
        // FEM3D Material Basis
        //$$
        // \begin{bmatrix}
        // \bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_0 \\
        // \end{bmatrix}^{-1}
        // $$
        muda::DeviceBuffer<Matrix3x3> Dm3x3_invs;


        // Kinetic Energy/Gradient/Hessian;
        muda::DeviceVar<Float> kinetic_energy;              // Kinetic Energy
        muda::DeviceBuffer<Float> vertex_kinetic_energies;  // Kinetic Energy Per Vertex
        muda::DeviceBuffer<Matrix3x3> H3x3s;  // size = vertex_count
        muda::DeviceBuffer<Vector3>   G3s;    // size = vertex_count

        // Elastic Energy/Gradient/Hessian;
        muda::DeviceVar<Float> codim_1d_elastic_energy;  // Codim1D Elastic Energy
        muda::DeviceBuffer<Float> codim_1d_elastic_energies;  // Codim1D Elastic Energy Per Element
        muda::DeviceBuffer<Vector6>   G6s;    // Codim1D Elastic Gradient
        muda::DeviceBuffer<Matrix6x6> H6x6s;  // Codim1D Elastic Hessian

        muda::DeviceVar<Float> codim_2d_elastic_energy;  // Codim2D Elastic Energy
        muda::DeviceBuffer<Float> codim_2d_elastic_energies;  // Codim2D Elastic Energy Per Element
        muda::DeviceBuffer<Vector9>   G9s;    // Codim2D Elastic Gradient
        muda::DeviceBuffer<Matrix9x9> H9x9s;  // Codim2D Elastic Hessian


        muda::DeviceVar<Float> fem_3d_elastic_energy;  // FEM3D Elastic Energy
        muda::DeviceBuffer<Float> fem_3d_elastic_energies;  // FEM3D Elastic Energy Per Element
        muda::DeviceBuffer<Vector12>    G12s;     // FEM3D Elastic Gradient
        muda::DeviceBuffer<Matrix12x12> H12x12s;  // FEM3D Elastic Hessian
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, SizeT consitution_index, Float dt) noexcept
            : m_impl(impl)
            , m_consitution_index(consitution_index)
            , m_dt(dt)
        {
        }

        Float dt() const noexcept;

      private:
        friend class Impl;
        SizeT m_consitution_index = ~0ull;
        Impl* m_impl              = nullptr;
        Float m_dt                = 0.0;
    };

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(Float dt) noexcept;

        auto dt() const noexcept { return m_dt; }

      private:
        friend class Impl;
        Float m_dt = 0.0;
    };


  public:
    void add_constitution(FiniteElementConstitution* constitution);

    // public data accessors:
    auto codim_0ds() const noexcept { return m_impl.codim_0ds.view(); }
    auto codim_1ds() const noexcept { return m_impl.codim_1ds.view(); }
    auto codim_2ds() const noexcept { return m_impl.codim_2ds.view(); }
    auto tets() const noexcept { return m_impl.tets.view(); }

    auto is_fixed() const noexcept { return m_impl.is_fixed.view(); }
    auto x_bars() const noexcept { return m_impl.x_bars.view(); }
    auto xs() const noexcept { return m_impl.xs.view(); }
    auto dxs() const noexcept { return m_impl.dxs.view(); }
    auto x_temps() const noexcept { return m_impl.x_temps.view(); }
    auto vs() const noexcept { return m_impl.vs.view(); }
    auto x_tildes() const noexcept { return m_impl.x_tildes.view(); }
    auto x_prevs() const noexcept { return m_impl.x_prevs.view(); }
    auto masses() const noexcept { return m_impl.masses.view(); }

    auto diag_hessians() const noexcept { return m_impl.diag_hessians.view(); }

    auto Dm3x3_invs() const noexcept { return m_impl.Dm3x3_invs.view(); }

    auto G3s() const noexcept { return m_impl.G3s.view(); }
    auto H3x3s() const noexcept { return m_impl.H3x3s.view(); }
    auto G6s() const noexcept { return m_impl.G6s.view(); }
    auto H6x6s() const noexcept { return m_impl.H6x6s.view(); }
    auto G9s() const noexcept { return m_impl.G9s.view(); }
    auto H9x9s() const noexcept { return m_impl.H9x9s.view(); }
    auto G12s() const noexcept { return m_impl.G12s.view(); }
    auto H12x12s() const noexcept { return m_impl.H12x12s.view(); }

  private:
    friend class FiniteElementVertexReporter;
    friend class FiniteElementSurfaceReporter;
    friend class FEMLinearSubsystem;
    friend class FEMLineSearchReporter;
    friend class FEMGradientHessianComputer;
    friend class FiniteElementConstitution;

    virtual void do_build() override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/finite_element_method.inl"