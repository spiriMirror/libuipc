#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <affine_body/abd_jacobi_matrix.h>
#include <uipc/geometry/simplicial_complex.h>
#include <sim_engine.h>
#include <dof_predictor.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class AffineBodyConstitution;
class AffineBodyEnergyProducer;
class AffineBodyDynamics : public SimSystem
{
    template <typename T>
    using DeviceBuffer = muda::DeviceBuffer<T>;

    template <typename T>
    using DeviceVar = muda::DeviceVar<T>;

  public:
    using SimSystem::SimSystem;

    class Impl;

    class ForEachInfo
    {
      public:
        SizeT global_index() const noexcept { return m_global_index; }
        SizeT local_index() const noexcept { return m_local_index; }

      private:
        friend class AffineBodyDynamics;
        SizeT m_global_index = 0;
        SizeT m_local_index  = 0;
    };

    class GeoInfo
    {
      public:
        IndexT geo_slot_index     = -1;
        U64    constitution_uid   = 0;
        U64    constitution_index = 0;

        SizeT vertex_offset = 0;
        SizeT vertex_count  = 0;

        SizeT body_offset = 0;
        SizeT body_count  = 0;
    };

    class ConstitutionInfo
    {
      public:
        SizeT geo_offset = 0;
        SizeT geo_count  = 0;

        SizeT body_offset = 0;
        SizeT body_count  = 0;

        SizeT vertex_offset = 0;
        SizeT vertex_count  = 0;
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SizeT constitution_index) noexcept;

        /**
         * @brief Short-cut to traverse all bodies of current constitution.
         * 
         * @code
         *  for_each(geo_slots, 
         *  [](SimplicialComplex& sc)
         *  {
         *      return sc.transforms().view();
         *  },
         *  [](const ForEachInfo& I, const Matrix4x4& transforms)
         *  {
         *      auto bodyI = I.global_index();
         *      ...
         *  })
         * @endcode
         */
        template <typename ViewGetterF, typename ForEachF>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetterF&&                   getter,
                      ForEachF&&                      for_each) const;


        /**
         * @brief Short-cut to traverse all geometries of current constitution.
         * 
         * @code
         *  for_each(geo_slots, 
         *  [](const ForEachInfo& I, SimplicialComplex& sc)
         *  {
         *      auto geoI = I.global_index();
         *      ...
         *  });
         * @endcode
         */
        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ForEachGeometry&&               for_every_geometry) const;

        span<const GeoInfo> geo_infos() const noexcept;

        const ConstitutionInfo& constitution_info() const noexcept;

        SizeT body_count() const noexcept;

        SizeT vertex_count() const noexcept;

      private:
        friend class Impl;
        SizeT m_constitution_index = ~0ull;
        Impl* m_impl               = nullptr;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, SizeT constitution_index, Float dt) noexcept;
        auto dt() const noexcept { return m_dt; }

        muda::CBufferView<Vector12> qs() const noexcept;
        muda::CBufferView<Float>    volumes() const noexcept;
        muda::BufferView<Float>     body_shape_energies() const noexcept;

      private:
        friend class Impl;
        SizeT m_constitution_index = ~0ull;
        Float m_dt                 = 0.0;
        Impl* m_impl               = nullptr;
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

        void _build_constitutions(WorldVisitor& world);
        void _build_geo_infos(WorldVisitor& world);

        void _setup_geometry_attributes(WorldVisitor& world);
        void _build_geometry_on_host(WorldVisitor& world);
        void _build_geometry_on_device(WorldVisitor& world);
        void _distribute_geo_infos();

        void write_scene(WorldVisitor& world);
        void _download_geometry_to_host();

        void compute_q_tilde(DofPredictor::PredictInfo& info);
        void compute_q_v(DofPredictor::ComputeVelocityInfo& info);

        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        template <typename ViewGetterF, typename ForEachF>
        static void _for_each(span<S<geometry::GeometrySlot>> geo_slots,
                              span<const GeoInfo>             geo_infos,
                              ViewGetterF&&                   getter,
                              ForEachF&&                      for_each);

        /*
         * @brief Short-cut to traverse all bodies of current constitution.
         */
        template <typename ViewGetterF, typename ForEachF>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetterF&&                   getter,
                      ForEachF&&                      for_each);

        template <typename ForEachGeometry>
        static void _for_each(span<S<geometry::GeometrySlot>> geo_slots,
                              span<const GeoInfo>             geo_infos,
                              ForEachGeometry&& for_every_geometry);

        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ForEachGeometry&&               for_every_geometry);

        SizeT geo_count() const noexcept { return abd_geo_count; }
        SizeT body_count() const noexcept { return abd_body_count; }
        SizeT vertex_count() const noexcept { return abd_vertex_count; }

        SimSystemSlotCollection<AffineBodyEnergyProducer> energy_producers;
        SimSystemSlotCollection<AffineBodyConstitution>   constitutions;
        unordered_map<U64, IndexT> constitution_uid_to_index;

        SizeT abd_geo_count    = 0;
        SizeT abd_body_count   = 0;
        SizeT abd_vertex_count = 0;

        // core invariant data
        vector<GeoInfo>          geo_infos;
        vector<ConstitutionInfo> constitution_infos;


        /******************************************************************************
        *                        host simulation data
        *******************************************************************************/
        vector<ABDJacobi> h_vertex_id_to_J;
        vector<IndexT>    h_vertex_id_to_body_id;
        vector<IndexT>    h_vertex_id_to_contact_element_id;
        // vector<Float>     h_vertex_id_to_mass;

        vector<Vector12>            h_body_id_to_q;
        vector<ABDJacobiDyadicMass> h_body_id_to_abd_mass;
        vector<Matrix12x12>         h_body_id_to_abd_mass_inv;
        vector<Float>               h_body_id_to_volume;
        vector<Vector12>            h_body_id_to_abd_gravity;
        vector<IndexT>              h_body_id_to_is_fixed;
        vector<IndexT>              h_body_id_to_is_dynamic;
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

        DeviceBuffer<IndexT> body_id_to_is_fixed;    // Body IsFixed
        DeviceBuffer<IndexT> body_id_to_is_dynamic;  // Body IsKinematic

        //tex: $$K_i$$ kinetic energy per body
        DeviceBuffer<Float> body_id_to_kinetic_energy;

        //tex: $$K$$
        DeviceVar<Float> abd_kinetic_energy;

        //tex: $$E$$
        DeviceBuffer<Float> body_id_to_shape_energy;
        DeviceVar<Float>    abd_shape_energy;

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

        BufferDump dump_q;
        BufferDump dump_q_v;
        BufferDump dump_q_prev;
    };

  public:
    /**
     * @brief affine body local vertex id to ABD Jacobi matrix
     */
    auto Js() const noexcept { return m_impl.vertex_id_to_J.view(); }
    /**
     * @brief affine body local vertex id to body id
     */
    auto v2b() const noexcept { return m_impl.vertex_id_to_body_id.view(); }

    auto qs() const noexcept { return m_impl.body_id_to_q.view(); }

    auto q_prevs() const noexcept { return m_impl.body_id_to_q_prev.view(); }

    auto q_vs() const noexcept { return m_impl.body_id_to_q_v.view(); }

    auto body_is_fixed() const noexcept
    {
        return m_impl.body_id_to_is_fixed.view();
    }

    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry);

  private:
    friend class AffineBodyConstitution;
    void add_constitution(AffineBodyConstitution* constitution);  // only be called by AffineBodyConstitution

    friend class AffineBodyVertexReporter;
    friend class AffinebodySurfaceReporter;

    friend class ABDLinearSubsystem;
    friend class ABDLineSearchReporter;
    friend class ABDDiagPreconditioner;

    friend class AffineBodyConstitution;
    friend class ABDGradientHessianComputer;
    friend class AffineBodyAnimator;

    friend class AffineBodyEnergyProducer;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/affine_body_dynamics.inl"
