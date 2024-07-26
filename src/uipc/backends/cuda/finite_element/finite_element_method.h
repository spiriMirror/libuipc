#pragma once
#include <sim_system.h>
#include <muda/buffer.h>
#include <dof_predictor.h>
#include <gradient_hessian_computer.h>

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
        SizeT geo_info_offset = ~0ull;
        SizeT geo_info_count  = 0ull;
        SizeT primitive_count = 0ull;
    };

    class FEM3DFilteredInfo
    {
      public:
        FEM3DFilteredInfo(Impl* impl) noexcept;

        span<const GeoInfo> geo_infos() const noexcept;

      private:
        SizeT m_index;
        Impl* m_impl;
    };

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
        void _distribute_constitution_infos();


        void compute_gradient_and_hessian(GradientHessianComputer::ComputeInfo&);


        void write_scene(WorldVisitor& world);

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
        vector<Vector3> h_positions;
        vector<Vector3> h_rest_positions;
        vector<Vector3> h_mass;

        vector<IndexT>   h_codim_0ds;
        vector<Vector2i> h_codim_1ds;
        vector<Vector3i> h_codim_2ds;
        vector<Vector4i> h_tets;

        muda::DeviceBuffer<IndexT>   codim_0ds;
        muda::DeviceBuffer<Vector2i> codim_1ds;
        muda::DeviceBuffer<Vector3i> codim_2ds;
        muda::DeviceBuffer<Vector4i> tets;

        muda::DeviceBuffer<Vector3> x_bar;    // Rest Positions
        muda::DeviceBuffer<Vector3> x;        // Positions
        muda::DeviceBuffer<Vector3> dx;       // Displacements
        muda::DeviceBuffer<Vector3> x_temp;   // Safe Positions for line search
        muda::DeviceBuffer<Vector3> v;        // Velocities
        muda::DeviceBuffer<Vector3> x_tilde;  // Predicted Positions
        muda::DeviceBuffer<Vector3> mass;     // Mass

        // Kinetic Energy Per Vertex
        muda::DeviceBuffer<Float> vertex_kinetic_energies;
        // Elastic Potential Energy Per Element
        muda::DeviceBuffer<Float> elastic_potential_energy;


        //tex:
        // FEM3D Material Basis
        //$$
        // \begin{bmatrix}
        // \bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_0 \\
        // \end{bmatrix}^{-1}
        // $$
        muda::DeviceBuffer<Matrix9x9> Dm9x9_inv;

        //  Elastic Hessian and Gradient:
        muda::DeviceBuffer<Matrix12x12> H12x12;  // FEM3D Elastic Hessian
        muda::DeviceBuffer<Vector9>     G12;     // FEM3D Elastic Gradient
        muda::DeviceBuffer<Matrix6x6>   H9x9;    // Codim2D Elastic Hessian
        muda::DeviceBuffer<Vector6>     G9;      // Codim2D Elastic Gradient
        muda::DeviceBuffer<Matrix3x3>   H6x6;    // Codim1D Elastic Hessian
        muda::DeviceBuffer<Vector3>     G6;      // Codim1D Elastic Gradient
    };

  public:
    void add_constitution(FiniteElementConstitution* constitution);

  private:
    Impl m_impl;

    virtual void do_build() override;
};
}  // namespace uipc::backend::cuda
