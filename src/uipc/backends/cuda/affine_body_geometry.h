#pragma once
#include <sim_system.h>
#include <global_vertex_manager.h>
#include <muda/buffer/device_buffer.h>
namespace uipc::backend::cuda
{
class ABDJacobi;
class ABDJacobiDyadicMass;
class AffineBodyGeometry : public SimSystem
{
    template <typename T>
    using DeviceBuffer = muda::DeviceBuffer<T>;

  public:
    using SimSystem::SimSystem;

  protected:
    virtual void build() override;

  private:
    GlobalVertexManager* m_global_vertex_manager = nullptr;


    vector<P<geometry::AttributeSlot<Matrix4x4>>> m_transforms;
    vector<Vector12>                              m_h_q;

    /******************************************************************************
    *                        abd point attributes
    *******************************************************************************/

    //tex: $$ \mathbf{J}(\bar{\mathbf{x}})
    // =
    //\left[\begin{array}{ccc|ccc:ccc:ccc}
    //1 &   &   & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  &  &  & \\
            //& 1 &   &  &  &  & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  \\
            //&   & 1 &  &  &  &  &  &  &  \bar{x}_1 & \bar{x}_2 & \bar{x}_3\\
            //\end{array}\right] $$
    DeviceBuffer<ABDJacobi> unique_point_id_to_J;

    DeviceBuffer<Float> unique_point_id_to_mass;


    /******************************************************************************
    *                        abd tetrahedron attributes
    *******************************************************************************
    * abd tetrahedron attributes are used to rebuild the abd system,
    * when some affine bodies are broken up.
    *******************************************************************************/
    //tex:
    //used to rebuild the abd body mass
    //$$
    //\mathbf{m} _j =\sum_ { k \in \mathcal{T}_j }
    //\mathbf{J} _k^T m_k \mathbf{J}_k
    //$$ where $\mathcal{T} _j$ is the tetrahedron with id $j$,
    //and $m_k$ is the mass of the vertex $k$.
    DeviceBuffer<ABDJacobiDyadicMass> tet_id_to_abd_mass;

    //tex:
    //used to rebuild the abd body force
    //$$
    //\mathbf{f}_j=\sum_{k \in \mathcal{T}_j}
    //\mathbf{J}_k^T f_k
    //$$
    //where $\mathcal{T}_j$ is the tetrahedron with id $j$.
    //and $f_k$ is the force of the vertex $k$.
    DeviceBuffer<Vector12> tet_id_to_abd_force;
    DeviceBuffer<Vector12> tet_id_to_abd_gravity_force;

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

    void report_vertex_count(VertexCountInfo& vertex_count_info);
    void receive_global_vertex_info(const GlobalVertexInfo& global_vertex_info);
    void init_affine_body_geometry();
};
}  // namespace uipc::backend::cuda
