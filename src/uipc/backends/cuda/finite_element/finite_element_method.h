#pragma once
#include <sim_system.h>
#include <muda/buffer.h>

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

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _init_constitutions();
        void _init_fem_geo_infos(WorldVisitor& world);
        void _build_on_host(WorldVisitor& world);
        void _build_on_device();
        void _download_geometry_to_host();
        void write_scene(WorldVisitor& world);

        // core invariant data:
        vector<GeoInfo>                                    fem_geo_infos;
        SimSystemSlotCollection<FiniteElementConstitution> constitutions;


        // related data:
        std::array<SizeT, 4> dim_geo_info_offsets;  // Map from dim to geo_info offset
        std::array<SizeT, 4> dim_geo_info_counts;  // Map from dim to geo_info count
        vector<FEM3DConstitution*>   fem_3d_constitutions;
        vector<Codim2DConstitution*> codim_2d_constitutions;
        vector<Codim1DConstitution*> codim_1d_constitutions;
        vector<Codim0DConstitution*> codim_0d_constitutions;


        // simulation data:
        vector<Vector3> h_positions;
        vector<Vector3> h_mass;

        vector<IndexT>   h_codim_0ds;
        vector<Vector2i> h_codim_1ds;
        vector<Vector3i> h_codim_2ds;
        vector<Vector4i> h_tets;

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
        //material basis
        //$$
        // \begin{bmatrix}
        // \bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_0 \\
        // \end{bmatrix}^{-1}
        // $$
        muda::DeviceBuffer<Matrix9x9> Dm_inv;

        //tex: spatial basis
        //$$
        // \begin{bmatrix}
        // \mathbf{x}_1 - \mathbf{x}_0 & \mathbf{x}_2 - \mathbf{x}_0 & \mathbf{x}_3 - \mathbf{x}_0 \\
        // \end{bmatrix}
        //$$
        muda::DeviceBuffer<Matrix9x9> Ds;

        //  Elastic Hessian and Gradient:
        muda::DeviceBuffer<Matrix9x9> H9x9;  // FEM3D Elastic Hessian
        muda::DeviceBuffer<Vector9>   G9;    // FEM3D Elastic Gradient
        muda::DeviceBuffer<Matrix6x6> H6x6;  // Codim2D Elastic Hessian
        muda::DeviceBuffer<Vector6>   G6;    // Codim2D Elastic Gradient
        muda::DeviceBuffer<Matrix3x3> H3x3;  // Codim1D Elastic Hessian
        muda::DeviceBuffer<Vector3>   G3;    // Codim1D Elastic Gradient
    };

  public:
    void add_constitution(FiniteElementConstitution* constitution);

  private:
    Impl m_impl;

    virtual void do_build() override;
};
}  // namespace uipc::backend::cuda
