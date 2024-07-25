#pragma once
#include <sim_system.h>
#include <muda/buffer.h>

namespace uipc::backend::cuda
{
class FiniteElementConstitution;

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
        void _init_fem_geo_infos(WorldVisitor& world);
        void _build_on_host(WorldVisitor& world);
        void _build_on_device();
        void _download_geometry_to_host();
        void write_scene(WorldVisitor& world);


        // core invariant data
        vector<GeoInfo> fem_geo_infos;

        SimSystemSlotCollection<FiniteElementConstitution> constitutions;

        vector<Vector3> h_positions;

        vector<IndexT>   h_codim_0ds;
        vector<Vector2i> h_codim_1ds;
        vector<Vector3i> h_codim_2ds;
        vector<Vector4i> h_tets;

        muda::DeviceBuffer<Vector3> positions;

        // FEM 3D
        muda::DeviceBuffer<Matrix12x12> H12x12;
        muda::DeviceBuffer<Vector12>    G12;

        // Codim 2D
        muda::DeviceBuffer<Matrix9x9> H9x9;
        muda::DeviceBuffer<Vector9>   G9;

        // Codim 1D
        muda::DeviceBuffer<Matrix6x6> H6x6;
        muda::DeviceBuffer<Vector6>   G6;

        // Codim 0D
        muda::DeviceBuffer<Matrix3x3> H3x3;
        muda::DeviceBuffer<Vector3>   G3;
    };

  public:
    void add_constitution(FiniteElementConstitution* constitution);

  private:
    Impl m_impl;

    virtual void do_build() override;
};
}  // namespace uipc::backend::cuda
