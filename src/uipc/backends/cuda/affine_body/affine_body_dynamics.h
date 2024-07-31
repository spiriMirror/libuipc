#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <affine_body/abd_jacobi_matrix.h>
#include <uipc/geometry/simplicial_complex.h>
#include <sim_engine.h>
#include <dof_predictor.h>
#include <gradient_hessian_computer.h>

namespace uipc::backend::cuda
{
class AffineBodyConstitution;
class AffineBodyDynamics : public SimSystem
{
    template <typename T>
    using DeviceBuffer = muda::DeviceBuffer<T>;

    template <typename T>
    using DeviceVar = muda::DeviceVar<T>;

  public:
    using SimSystem::SimSystem;

    class Impl;

    class BodyInfo
    {
      public:
        U64    constitution_uid() const noexcept;
        IndexT geometry_slot_index() const noexcept;
        IndexT geometry_instance_index() const noexcept;
        IndexT abd_geometry_index() const noexcept;
        IndexT affine_body_id() const noexcept;
        IndexT vertex_offset() const noexcept;
        IndexT vertex_count() const noexcept;

      private:
        friend class Impl;
        U64    m_constitution_uid   = 0ull;
        IndexT m_constitution_index = -1;

        IndexT m_geometry_slot_index     = -1;
        IndexT m_geometry_instance_index = -1;

        IndexT m_abd_geometry_index = -1;
        IndexT m_affine_body_id     = -1;

        IndexT m_vertex_offset = -1;
        IndexT m_vertex_count  = -1;
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl) noexcept;

        span<const BodyInfo> body_infos() const noexcept;

        /**
         * @brief Short-cut to traverse all bodies of current constitution.
         *  
         * @param getter f: `span<T>(SimplicialComplex&)` or `span<const T>(SimplicialComplex&)`
         * @param for_each f: `void(SizeT,T&)` or `void(SizeT,const T&)`
         */
        template <typename ViewGetterF, typename ForEachF>
        void for_each_body(span<S<geometry::GeometrySlot>> geo_slots,
                           ViewGetterF&&                   getter,
                           ForEachF&&                      for_each) const;

        static geometry::SimplicialComplex& geometry(span<S<geometry::GeometrySlot>> geo_slots,
                                                     const BodyInfo& body_info);

      private:
        friend class Impl;
        SizeT m_constitution_index = ~0ull;
        Impl* m_impl               = nullptr;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(Impl*                impl,
                          SizeT                constitution_index,
                          muda::VarView<Float> shape_energy,
                          Float                dt) noexcept;
        auto shape_energy() const noexcept { return m_shape_energy; }
        auto dt() const noexcept { return m_dt; }

        muda::CBufferView<Vector12> qs() const noexcept;
        muda::CBufferView<Float>    volumes() const noexcept;

      private:
        friend class Impl;
        SizeT                m_constitution_index = ~0ull;
        muda::VarView<Float> m_shape_energy;
        Float                m_dt   = 0.0;
        Impl*                m_impl = nullptr;
    };

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   SizeT constitution_index,
                                   muda::BufferView<Vector12>    shape_gradient,
                                   muda::BufferView<Matrix12x12> shape_hessian,
                                   Float                         dt) noexcept;

        auto shape_hessian() const noexcept { return m_shape_hessian; }
        auto shape_gradient() const noexcept { return m_shape_gradient; }
        muda::CBufferView<Vector12> qs() const noexcept;
        muda::CBufferView<Float>    volumes() const noexcept;
        auto                        dt() const noexcept { return m_dt; }

      private:
        friend class Impl;
        SizeT                         m_constitution_index = ~0ull;
        muda::BufferView<Matrix12x12> m_shape_hessian;
        muda::BufferView<Vector12>    m_shape_gradient;
        Float                         m_dt   = 0.0;
        Impl*                         m_impl = nullptr;
    };

    void add_constitution(AffineBodyConstitution* constitution);

    void after_build_geometry(SimSystem& sim_system, std::function<void()>&& action);

  protected:
    virtual void do_build() override;

    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;
  public:
    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _build_subsystems(WorldVisitor& world);
        void _build_body_infos(WorldVisitor& world);
        void _build_related_infos(WorldVisitor& world);
        void _build_geometry_on_host(WorldVisitor& world);
        void _build_geometry_on_device(WorldVisitor& world);
        void _distribute_body_infos();

        void write_scene(WorldVisitor& world);
        void _download_geometry_to_host();

        void compute_q_tilde(DoFPredictor::PredictInfo& info);
        void compute_q_v(DoFPredictor::PredictInfo& info);
        void compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info);


        // util functions
        static geometry::SimplicialComplex& geometry(span<S<geometry::GeometrySlot>> geo_slots,
                                                     const BodyInfo& body_info);

        /*
        * @brief Short-cut to traverse all bodies of current constitution.
        * 
        * @param getter f: `span<T>(SimplicialComplex&)` or `span<const T>(SimplicialComplex&)`
        * @param for_each f: `void(SizeT,T&)` or `void(SizeT,const T&)`
        */
        template <typename ViewGetterF, typename ForEachF>
        void for_each_body(span<S<geometry::GeometrySlot>> geo_slots,
                           ViewGetterF&&                   getter,
                           ForEachF&&                      for_each);


        SizeT body_count() const noexcept { return h_body_infos.size(); }

      public:
        SimActionCollection<void()>                     after_build_geometry;
        SimSystemSlotCollection<AffineBodyConstitution> constitutions;

        // core invariant data
        vector<BodyInfo> h_body_infos;

        // related cache of `h_body_infos`
        SizeT abd_geo_count    = 0;
        SizeT abd_body_count   = 0;
        SizeT abd_vertex_count = 0;

        vector<SizeT> h_constitution_geo_offsets;
        vector<SizeT> h_constitution_geo_counts;
        vector<SizeT> h_abd_geo_body_offsets;
        vector<SizeT> h_abd_geo_body_counts;
        vector<SizeT> h_constitution_body_offsets;
        vector<SizeT> h_constitution_body_counts;


        /******************************************************************************
        *                        host simulation data
        *******************************************************************************/
        vector<ABDJacobi>           h_vertex_id_to_J;
        vector<IndexT>              h_vertex_id_to_body_id;
        vector<IndexT>              h_vertex_id_to_contact_element_id;
        vector<Float>               h_vertex_id_to_mass;
        vector<Vector12>            h_body_id_to_q;
        vector<ABDJacobiDyadicMass> h_body_id_to_abd_mass;
        vector<Matrix12x12>         h_body_id_to_abd_mass_inv;
        vector<Float>               h_body_id_to_volume;
        vector<Vector12>            h_body_id_to_abd_gravity;
        vector<IndexT>              h_body_id_to_is_fixed;
        vector<Float>               h_constitution_shape_energy;

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
        //used to rebuild the affine_body_dynamics shape energy coefficient
        //$$V_{\perp}(m_q)=\kappa v_b\left\|\mathrm{AA}^{T}-\mathrm{I}_{3}\right\|_{F}^{2}$$
        //where $v_b$ is the volume of the affine body.
        DeviceBuffer<Float> body_id_to_volume;

        //tex:
        // $$\mathbf{m_q} =
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

        //tex: temp $\mathbf{q}$ for line search rollback
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

        //tex: simple boundary condition
        DeviceBuffer<IndexT> body_id_to_is_fixed;

        //tex: $$K_i$$ kinetic energy per body
        DeviceBuffer<Float> body_id_to_kinetic_energy;

        //tex: $$K$$
        DeviceVar<Float> abd_kinetic_energy;

        //tex: $$E$$
        DeviceBuffer<Float> constitution_shape_energy;

        //tex: $$ \mathbf{H}_{ii} + \mathbf{M}_{ii} $$
        DeviceBuffer<Matrix12x12> body_id_to_body_hessian;
        //tex: $$ \mathbf{g}_{i} $$
        DeviceBuffer<Vector12> body_id_to_body_gradient;

        //tex: consider contact
        DeviceBuffer<Matrix12x12> diag_hessian;

        template <typename T>
        muda::BufferView<T> subview(DeviceBuffer<T>& body_id_to_values,
                                    SizeT constitution_index) const noexcept;

        template <typename T>
        span<T> subview(vector<T>& body_id_to_values, SizeT constitution_index) const noexcept;
    };

  private:
    friend class AffineBodyVertexReporter;
    friend class AffinebodySurfaceReporter;

    friend class ABDLinearSubsystem;
    friend class ABDLineSearchReporter;
    friend class ABDDiagPreconditioner;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/affine_body_dynamics.inl"
