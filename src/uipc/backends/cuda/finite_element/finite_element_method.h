#pragma once
#include <sim_system.h>

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

        IndexT index = -1;

        SizeT vertex_offset   = ~0ull;
        SizeT vertex_count    = 0ull;
        SizeT codim_0d_offset = ~0ull;
        SizeT codim_0d_count  = 0ull;
        SizeT codim_1d_offset = ~0ull;
        SizeT codim_1d_count  = 0ull;
        SizeT codim_2d_offset = ~0ull;
        SizeT codim_2d_count  = 0ull;
        SizeT tet_offset      = ~0ull;
        SizeT tet_count       = 0ull;
    };

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _init_fem_geo_infos(WorldVisitor& world);
        void write_scene(WorldVisitor& world);


        // core invariant data
        vector<GeoInfo> fem_geo_infos;

        SimSystemSlotCollection<FiniteElementConstitution> constitutions;


        vector<Vector3>  h_positions;
        vector<IndexT>   h_codim_0ds;
        vector<Vector2i> h_codim_1ds;
        vector<Vector3i> h_codim_2ds;
        vector<Vector4i> h_tets;
    };

  private:
    Impl m_impl;

    virtual void do_build() override;
};
}  // namespace uipc::backend::cuda
