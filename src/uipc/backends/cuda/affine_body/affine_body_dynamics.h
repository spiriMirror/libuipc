#pragma once
#include <sim_system.h>
#include <global_vertex_manager.h>
#include <muda/buffer/device_buffer.h>
#include <affine_body/abd_jacobi_matrix.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::backend::cuda
{
class AffineBodyDynamics : public SimSystem
{
    template <typename T>
    using DeviceBuffer = muda::DeviceBuffer<T>;

  public:
    static U<AffineBodyDynamics> advanced_creator(SimEngine& engine);

    using SimSystem::SimSystem;

    class BodyInfo
    {
      public:
        U64    constitution_uid        = 0ull;
        IndexT global_geometry_index   = -1;
        IndexT abd_geometry_index      = -1;
        IndexT geometry_instance_index = -1;
        IndexT affine_body_id          = -1;
        IndexT vertex_offset           = -1;
        IndexT vertex_count            = -1;
    };

    class FilteredInfo
    {
      public:
      private:
        span<const BodyInfo> m_body_infos;
    };

    class Filter
    {
      public:
        Filter(U64 constitution_uid, std::function<void(const FilteredInfo&)>&& action) noexcept;

        U64  uid() const noexcept { return m_constitution_uid; }
        void operator()(const FilteredInfo& info) const { m_action(info); }

      private:
        U64                                      m_constitution_uid;
        std::function<void(const FilteredInfo&)> m_action;
    };

    void on_filter(Filter&& filter);

  protected:
    virtual void build() override;

  public:
    class Impl
    {
      public:
        void init_affine_body_geometry(WorldVisitor& world);
        void _find_affine_bodies(WorldVisitor& world);
        void _build_geometry_on_host(WorldVisitor& world);
        void _build_geometry_on_device(WorldVisitor& world);

        void report_vertex_count(VertexCountInfo& vertex_count_info);
        void receive_global_vertex_info(const GlobalVertexInfo& global_vertex_info);


        void write_scene(WorldVisitor& world);
        void _download_geometry_to_host();

        // util functions
        geometry::SimplicialComplex& geometry(span<P<geometry::GeometrySlot>> geo_slots,
                                              const BodyInfo body_info);

      public:
        GlobalVertexManager* global_vertex_manager = nullptr;

        vector<Filter> filters;
        SizeT          abd_global_vertex_offset = 0;
        SizeT          abd_global_vertex_count  = 0;
        SizeT          abd_geo_count            = 0;

        vector<BodyInfo> h_body_infos;
        vector<SizeT>    abd_geo_body_offsets;
        vector<SizeT>    abd_geo_body_counts;

        vector<geometry::AttributeSlot<Matrix4x4>*> transforms;

        vector<ABDJacobi>           h_vertex_id_to_J;
        vector<IndexT>              h_vertex_id_to_body_id;
        vector<Float>               h_vertex_id_to_mass;
        vector<Vector12>            h_body_id_to_q;
        vector<ABDJacobiDyadicMass> h_body_id_to_abd_mass;
        vector<Matrix12x12>         h_body_id_to_abd_mass_inv;
        vector<Float>               h_body_id_to_volume;
        vector<Vector12>            h_body_id_to_abd_gravity;

        /******************************************************************************
        *                        abd vertex attributes
        *******************************************************************************/
        //tex: $$ \mathbf{J}(\bar{\mathbf{x}})
        // =
        //\left[\begin{array}{ccc|ccc:ccc:ccc}
        //1 &   &   & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  &  &  & \\
            //& 1 &   &  &  &  & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  \\
            //&   & 1 &  &  &  &  &  &  &  \bar{x}_1 & \bar{x}_2 & \bar{x}_3\\
            //\end{array}\right] $$
        DeviceBuffer<ABDJacobi> vertex_id_to_J;
        DeviceBuffer<IndexT>    vertex_id_to_body_id;


        /******************************************************************************
        *                           abd body attributes
        *******************************************************************************
        * abd body attributes are something that involved into physics simulation
        *******************************************************************************/
        //tex:
        //$$
        //\mathbf{M}_i
        // =
        // \sum_{j \in \mathcal{B}_i} \mathbf{m}_j
        //$$
        //where $\mathcal{B}_i$ is the body $i$,
        //built from the tetrahedrons $\mathcal{T}_j$ it contains.
        DeviceBuffer<ABDJacobiDyadicMass> body_id_to_abd_mass;
        //tex: $$ \mathbf{M}_i^{-1} $$
        DeviceBuffer<Matrix12x12> body_id_to_abd_mass_inv;

        //tex:
        //used to rebuild the abd shape energy coefficient
        //$$V_{\perp}(q)=\kappa v_b\left\|\mathrm{AA}^{T}-\mathrm{I}_{3}\right\|_{F}^{2}$$
        //where $v_b$ is the volume of the affine body.
        DeviceBuffer<Float> body_id_to_volume;

        //tex:
        // $$\mathbf{q} =
        // \begin{bmatrix}
        // \mathbf { p } \\
            // \mathbf{a} _1 \\
            // \mathbf{a} _2 \\
            // \mathbf{a} _3 \\
            // \end{bmatrix}
        // $$
        // where
        // $$
        // A = \begin{bmatrix}
        // \mathbf{a}_1 & \mathbf{a}_2 & \mathbf{a}_3
        // \end{bmatrix} ^T
        // $$
        DeviceBuffer<Vector12> body_id_to_q;

        // temp q for line search rollback
        DeviceBuffer<Vector12> body_id_to_q_temp;

        //tex:
        //$$
        //\tilde{\mathbf{q}}_{b}=\mathbf{q}_{b}^{t}+\Delta t \dot{\mathbf{q}}_{b}^{t}+\Delta t^{2} \mathbf{M}^{-1} \mathbf{f}_{b}^{t+1}
        //$$
        //predicted q
        DeviceBuffer<Vector12> body_id_to_q_tilde;
        //tex: $$ \mathbf{q}^{t}_b $$
        DeviceBuffer<Vector12> body_id_to_q_prev;

        //tex: $$ \dot{\mathbf{q}} $$
        DeviceBuffer<Vector12> body_id_to_q_v;

        //tex: $$ \Delta\mathbf{q} $$
        //for move dir calculation, which means we use PCG/direct solver to solve it.
        DeviceBuffer<Vector12> body_id_to_dq;

        //tex:
        //$$
        //\mathbf{F}_i
        // =
        //\sum_{j \in \mathcal{B}_i} \mathbf{f}_j
        //$$
        // where $\mathcal{B}_i$ is the body $i$,
        // built from the tetrahedrons $\mathcal{T}_j$ it contains.
        DeviceBuffer<Vector12> body_id_to_abd_force;

        //tex:
        //$$
        //\mathbf{G}_i
        // =
        // \mathbf{M}_i^{-1}\left( \sum_{j \in \mathcal{B}_i} \sum_{k \in \mathcal{T}_j} \mathbf{J}_k^T m_k g_k\right)
        //$$
        DeviceBuffer<Vector12> body_id_to_abd_gravity;
    };

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/affine_body_dynamics.inl"
