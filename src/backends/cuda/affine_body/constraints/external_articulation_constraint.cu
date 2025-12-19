#include <affine_body/inter_affine_body_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <uipc/common/zip.h>
#include <affine_body/constraints/external_articulation_constraint_function.h>
#include <affine_body/utils.h>
#include <algorithm/fast_segmental_reduce.h>
#include <thrust/functional.h>
#include <utils/matrix_assembler.h>
#include <utils/matrix_unpacker.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
UIPC_GENERIC Vector2i joint_joint_id_to_ij(IndexT joint_joint_id,
                                           const muda::CDense1D<IndexT>& joint_joint_id_to_art_id,
                                           const muda::CDense1D<IndexT>& art_joint_offsets,
                                           const muda::CDense1D<IndexT>& art_joint_counts)
{
    auto art_id = joint_joint_id_to_art_id(joint_joint_id);
    auto offset = art_joint_offsets(art_id);
    auto count  = art_joint_counts(art_id);

    auto local_id = joint_joint_id - offset;
    auto index_i  = local_id / count;
    auto index_j  = local_id - index_i * count;
    return Vector2i{IndexT(index_i), IndexT(index_j)};
}

// Ref: libuipc/scripts/symbol_calculation/external_articulation_constraint.ipynb
class ExternalArticulationConstraint final : public InterAffineBodyConstraint
{
    static constexpr U64 ConstitutionUID             = 23ull;
    static constexpr U64 RevoluteJointConstraintUID  = 18ull;
    static constexpr U64 PrismaticJointConstraintUID = 20ull;

  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;


    vector<S<geometry::AttributeSlot<Float>>> h_art_joint_joint_mass;
    vector<S<geometry::AttributeSlot<Float>>> h_art_joint_delta_theta_tilde;

    OffsetCountCollection<IndexT> h_art_joint_offsets_counts;
    vector<IndexT>                h_joint_id_to_art_id;
    vector<U64>                   h_joint_id_to_uid;
    vector<Vector2i>              h_joint_id_to_body_ids;
    vector<Float>                 h_joint_id_to_delta_theta_tilde;
    vector<Float>                 h_joint_id_to_delta_theta;

    OffsetCountCollection<IndexT> h_art_joint_joint_offsets_counts;
    vector<IndexT>                h_joint_joint_id_to_art_id;
    vector<Float>                 h_joint_joint_id_to_mass;

    muda::DeviceBuffer<IndexT>   art_joint_offsets;
    muda::DeviceBuffer<IndexT>   art_joint_counts;
    muda::DeviceBuffer<IndexT>   joint_id_to_art_id;
    muda::DeviceBuffer<U64>      joint_id_to_uid;
    muda::DeviceBuffer<Vector2i> joint_id_to_body_ids;
    muda::DeviceBuffer<Float>    joint_id_to_delta_theta;
    muda::DeviceBuffer<Float>    joint_id_to_delta_theta_tilde;

    muda::DeviceBuffer<IndexT> art_joint_joint_offsets;
    muda::DeviceBuffer<IndexT> art_joint_joint_counts;
    muda::DeviceBuffer<IndexT> joint_joint_id_to_art_id;
    muda::DeviceBuffer<Float>  joint_joint_id_to_mass;

    vector<Vector6> h_joint_id_to_L_basis;
    vector<Vector6> h_joint_id_to_R_basis;

    // Prismatic Joint: [c_bar, t_bar]
    // Revolute Joint:  [bn_bar, tn_bar]
    muda::DeviceBuffer<Vector6> joint_id_to_L_basis;
    muda::DeviceBuffer<Vector6> joint_id_to_R_basis;

    muda::DeviceBuffer<Float> joint_id_to_G_theta;

    void do_build(BuildInfo& info) override {}

    U64 get_uid() const noexcept override { return ConstitutionUID; }

    auto get_revolute_basis(const geometry::SimplicialComplex* L,
                            IndexT                             L_inst_id,
                            const geometry::SimplicialComplex* R,
                            IndexT                             R_inst_id,
                            const geometry::SimplicialComplex* joint_mesh,
                            IndexT                             joint_index)
    {
        auto topo_view = joint_mesh->edges().topo().view();
        auto pos_view  = joint_mesh->positions().view();

        Vector2i e = topo_view[joint_index];
        Vector3  t = pos_view[e[1]] - pos_view[e[0]];
        Vector3  n;
        Vector3  b;
        orthonormal_basis(t, n, b);

        auto compute_bn_bar = [&](const geometry::SimplicialComplex* geo, IndexT inst_id) -> Vector6
        {
            UIPC_ASSERT(geo, "ExternalArticulationConstraint: Geometry is null when computing bn_bar");

            const Matrix4x4& trans = geo->transforms().view()[inst_id];
            Transform        T{trans};
            Matrix3x3        InvRot = T.rotation().inverse();
            Vector6          bn_bar;
            bn_bar.segment<3>(0) = InvRot * b;
            bn_bar.segment<3>(3) = InvRot * n;
            return bn_bar;
        };

        Vector6 L_bn_bar = compute_bn_bar(L, L_inst_id);
        Vector6 R_bn_bar = compute_bn_bar(R, R_inst_id);

        return std::tuple{L_bn_bar, R_bn_bar};
    }

    auto get_joint_info(span<S<geometry::GeometrySlot>>        geo_slots,
                        InterAffineBodyAnimator::FilteredInfo& info,
                        const geometry::Geometry&              mesh,
                        IndexT                                 index)
    {
        auto joint_mesh = mesh.as<geometry::SimplicialComplex>();
        UIPC_ASSERT(joint_mesh,
                    "ExternalArticulationConstraint: Joint geometry ({}) is not a SimplicialComplex",
                    joint_mesh->to_json().dump(4));

        UIPC_ASSERT(index < joint_mesh->edges().size() && index >= 0,
                    "ExternalArticulationConstraint: Joint index ({}) out of range in Joint Geometry ({}), [0, {}) expected",
                    index,
                    joint_mesh->to_json().dump(4),
                    joint_mesh->edges().size());

        auto uid_attr = joint_mesh->meta().find<U64>(builtin::constitution_uid);
        UIPC_ASSERT(uid_attr, "ExternalArticulationConstraint: 'constitution_uid' meta attribute not found in joint geometry");

        U64 uid_value = uid_attr->view()[0];
        UIPC_ASSERT(uid_value == RevoluteJointConstraintUID,
                    "ExternalArticulationConstraint: Now only support RevoluteJointConstraint (UID={}), but found joint with UID {}",
                    RevoluteJointConstraintUID,
                    uid_value);

        auto joint_geo_ids  = joint_mesh->edges().find<Vector2i>("geo_ids");
        auto joint_inst_ids = joint_mesh->edges().find<Vector2i>("inst_ids");
        auto joint_geo_ids_view  = joint_geo_ids->view();
        auto joint_inst_ids_view = joint_inst_ids->view();

        const Vector2i geo_id  = joint_geo_ids_view[index];
        const Vector2i inst_id = joint_inst_ids_view[index];

        Vector2i body_id = {
            info.body_id(geo_id[0], inst_id[0]),
            info.body_id(geo_id[1], inst_id[1]),
        };

        auto L_link_geo = info.body_geo(geo_slots, geo_id[0]);
        auto R_link_geo = info.body_geo(geo_slots, geo_id[1]);


        auto [L_basis, R_basis] = get_revolute_basis(
            L_link_geo, inst_id[0], R_link_geo, inst_id[1], joint_mesh, index);

        return std::tuple{uid_value, body_id, L_basis, R_basis};
    }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto scene     = world().scene();
        auto geo_slots = scene.geometries();

        auto art_count = info.anim_inter_geo_infos().size();

        h_art_joint_offsets_counts.resize(art_count);

        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                U64  uid_value = uid->view()[0];
                UIPC_ASSERT(uid_value == ConstitutionUID,
                            "ExternalArticulationConstraint: Geometry constitution UID mismatch, {} expected, {} found",
                            ConstitutionUID,
                            uid_value);

                auto joint_collection = geo["joint"];
                UIPC_ASSERT(joint_collection, "ExternalArticulationConstraint: 'joint' attribute collection not found in geometry");

                h_art_joint_offsets_counts.counts()[I.index()] =
                    joint_collection->size();

                auto geo_ids = joint_collection->find<IndexT>("geo_id");
                UIPC_ASSERT(geo_ids, "ExternalArticulationConstraint: 'geo_id' attribute not found in joint collection");
                auto geo_id_view = geo_ids->view();

                auto indices = joint_collection->find<IndexT>("index");
                UIPC_ASSERT(indices, "ExternalArticulationConstraint: 'index' attribute not found in joint collection");
                auto index_view = indices->view();

                auto delta_theta_tilde = joint_collection->find<Float>("delta_theta_tilde");
                UIPC_ASSERT(delta_theta_tilde,
                            "ExternalArticulationConstraint: 'delta_theta_tilde' attribute not found in joint collection");
                auto delta_theta_tilde_view = delta_theta_tilde->view();

                auto joint_joint_collection = geo["joint_joint"];
                UIPC_ASSERT(joint_joint_collection,
                            "ExternalArticulationConstraint: 'joint_joint' attribute collection not found in geometry");
                UIPC_ASSERT(joint_joint_collection->size()
                                == joint_collection->size() * joint_collection->size(),
                            "ExternalArticulationConstraint: 'joint_joint' collection size mismatch");

                auto mass = joint_joint_collection->find<Float>("mass");
                UIPC_ASSERT(mass, "ExternalArticulationConstraint: 'mass' attribute not found in joint_joint collection");

                auto mass_view = mass->view();

                h_art_joint_joint_offsets_counts.counts()[I.index()] = mass->size();

                for(auto&& [geo_id, index, delta_theta_tilde] :
                    zip(geo_id_view, index_view, delta_theta_tilde_view))
                {
                    // find the joint geometry in the scene
                    auto geo_slot = scene.find_geometry(geo_id);

                    auto [joint_uid, body_id, L_basis, R_basis] =
                        get_joint_info(geo_slots, info, geo_slot->geometry(), index);

                    h_joint_id_to_art_id.push_back(I.index());
                    h_joint_id_to_uid.push_back(joint_uid);
                    h_joint_id_to_body_ids.push_back(body_id);
                    h_joint_id_to_delta_theta_tilde.push_back(delta_theta_tilde);
                    h_joint_id_to_L_basis.push_back(L_basis);
                    h_joint_id_to_R_basis.push_back(R_basis);
                }

                for(auto&& m : mass_view)
                {
                    h_joint_joint_id_to_mass.push_back(m);
                }

                // theta_tilde and mass will change in each step, so we just prepare the slots here
                // later we will fill them in do_step()
                h_art_joint_delta_theta_tilde.push_back(delta_theta_tilde);
                h_art_joint_joint_mass.push_back(mass);
            });

        h_art_joint_offsets_counts.scan();
        h_art_joint_joint_offsets_counts.scan();


        // upload to device

        // Joint Info
        auto n_joint = h_joint_id_to_art_id.size();

        auto h_art_joint_offsets = h_art_joint_offsets_counts.offsets();
        art_joint_offsets.resize(h_art_joint_offsets.size());
        art_joint_offsets.view().copy_from(h_art_joint_offsets.data());

        auto h_art_joint_counts = h_art_joint_offsets_counts.counts();
        art_joint_counts.resize(h_art_joint_counts.size());
        art_joint_counts.view().copy_from(h_art_joint_counts.data());

        joint_id_to_art_id.resize(n_joint);
        joint_id_to_art_id.view().copy_from(h_joint_id_to_art_id.data());

        joint_id_to_uid.resize(h_joint_id_to_uid.size());
        joint_id_to_uid.view().copy_from(h_joint_id_to_uid.data());

        joint_id_to_body_ids.resize(h_joint_id_to_body_ids.size());
        joint_id_to_body_ids.view().copy_from(h_joint_id_to_body_ids.data());

        // init delta_theta to zero
        joint_id_to_delta_theta.resize(n_joint);
        joint_id_to_delta_theta.fill(0.0f);
        h_joint_id_to_delta_theta.resize(n_joint, 0.0f);

        // init delta_theta_tilde
        joint_id_to_delta_theta_tilde.resize(h_joint_id_to_delta_theta_tilde.size());
        joint_id_to_delta_theta_tilde.view().copy_from(
            h_joint_id_to_delta_theta_tilde.data());

        joint_id_to_L_basis.resize(h_joint_id_to_L_basis.size());
        joint_id_to_L_basis.view().copy_from(h_joint_id_to_L_basis.data());

        joint_id_to_R_basis.resize(h_joint_id_to_R_basis.size());
        joint_id_to_R_basis.view().copy_from(h_joint_id_to_R_basis.data());

        joint_id_to_G_theta.resize(n_joint);

        // Joint-Joint Info
        auto h_art_mass_offsets = h_art_joint_counts;
        art_joint_joint_offsets.resize(h_art_mass_offsets.size());
        art_joint_joint_offsets.view().copy_from(h_art_mass_offsets.data());

        auto h_art_mass_counts = h_art_joint_joint_offsets_counts.counts();
        art_joint_joint_counts.resize(h_art_mass_counts.size());
        art_joint_joint_counts.view().copy_from(h_art_mass_counts.data());

        joint_joint_id_to_mass.resize(h_joint_joint_id_to_mass.size());
        joint_joint_id_to_mass.view().copy_from(h_joint_joint_id_to_mass.data());
    }

    void do_step(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        // only collect mass and delta_theta_tilde
        auto geo_slots = world().scene().geometries();

        h_joint_joint_id_to_mass.clear();
        for(auto&& mass : h_art_joint_joint_mass)
        {
            auto mass_view = mass->view();
            std::ranges::copy(mass_view, std::back_inserter(h_joint_joint_id_to_mass));
        }

        h_joint_id_to_delta_theta_tilde.clear();
        for(auto&& delta_theta_tilde : h_art_joint_delta_theta_tilde)
        {
            auto delta_theta_tilde_view = delta_theta_tilde->view();
            std::ranges::copy(delta_theta_tilde_view,
                              std::back_inserter(h_joint_id_to_delta_theta_tilde));
        }

        // upload to device
        auto async_copy = []<typename T>(muda::DeviceBuffer<T>& device_buffer,
                                         span<const T>          host_vector)
        {
            muda::BufferLaunch().copy<T>(device_buffer.view(), host_vector.data());
        };

        async_copy(joint_joint_id_to_mass, span<const Float>{h_joint_joint_id_to_mass});
        async_copy(joint_id_to_delta_theta_tilde,
                   span<const Float>{h_joint_id_to_delta_theta_tilde});
    }

    // m_ij ->

    void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override
    {
        auto e_count = joint_joint_id_to_mass.size();
        info.energy_count(e_count);
        auto g_count = 0;  // TODO
        info.gradient_segment_count(g_count);
        auto h_count = 0;  // TODO
        info.hessian_block_count(h_count);
    }

    void do_compute_energy(InterAffineBodyAnimator::EnergyInfo& info) override
    {
        using namespace muda;

        namespace ERJ = sym::external_revolute_joint_constraint;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                joint_joint_id_to_mass.size(),
                [energies = info.energies().viewer().name("energies"),
                 joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_id_to_art_id = joint_id_to_art_id.cviewer().name("joint_id_to_art_id"),
                 art_joint_offsets = art_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_joint_counts.cviewer().name("art_joint_counts"),
                 joint_id_to_delta_theta = joint_id_to_delta_theta.cviewer().name("joint_id_to_delta_theta"),
                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),
                 joint_id_to_body_ids = joint_id_to_body_ids.cviewer().name("joint_id_to_body_ids"),


                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 qs = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs")] __device__(IndexT I) mutable
                {
                    Float m_ij = joint_joint_id_to_mass(I);

                    Vector2i ij = joint_joint_id_to_ij(
                        I, joint_id_to_art_id, art_joint_offsets, art_joint_counts);

                    auto compute_delta_theta = [&](IndexT joint_id) -> Float
                    {
                        Vector6  L_basis  = joint_id_to_L_basis(joint_id);
                        Vector6  R_basis  = joint_id_to_R_basis(joint_id);
                        Vector2i body_ids = joint_id_to_body_ids(joint_id);

                        const Vector12& qk      = qs(body_ids[0]);
                        const Vector12& ql      = qs(body_ids[1]);
                        const Vector12& q_prevk = q_prevs(body_ids[0]);
                        const Vector12& q_prevl = q_prevs(body_ids[1]);

                        Vector12 F;
                        ERJ::F<Float>(F, L_basis, qk, R_basis, ql);
                        Vector12 F_t;
                        ERJ::F<Float>(F_t, L_basis, q_prevk, R_basis, q_prevl);

                        Float delta_theta;
                        ERJ::DeltaTheta(delta_theta, F, F_t);

                        return delta_theta;
                    };


                    Float delta_theta_i = compute_delta_theta(ij[0]);
                    Float delta_theta_j = compute_delta_theta(ij[1]);

                    Float delta_theta_tilde_i = joint_id_to_delta_theta_tilde(ij[0]);
                    Float delta_theta_tilde_j = joint_id_to_delta_theta_tilde(ij[1]);

                    energies(I) = 0.5 * (delta_theta_i - delta_theta_tilde_i)
                                  * m_ij * (delta_theta_j - delta_theta_tilde_j);
                });
    }

    void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override
    {
        using namespace muda;

        // Compute G^theta
        FastSegmentalReduce()
            .file_line(__FILE__, __LINE__)
            .reduce(
                joint_joint_id_to_mass.size(),
                joint_id_to_G_theta.view(),
                [joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_id_to_art_id = joint_id_to_art_id.cviewer().name("joint_id_to_art_id"),
                 art_joint_offsets = art_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_joint_counts.cviewer().name(
                     "art_joint_counts")] __device__(IndexT I) -> IndexT
                {
                    Vector2i ij = joint_joint_id_to_ij(
                        I, joint_id_to_art_id, art_joint_offsets, art_joint_counts);
                    return ij[0];
                },
                [joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_id_to_art_id = joint_id_to_art_id.cviewer().name("joint_id_to_art_id"),
                 art_joint_offsets = art_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_joint_counts.cviewer().name(
                     "art_joint_counts")] __device__(IndexT I) -> Float
                {
                    Vector2i ij = joint_joint_id_to_ij(
                        I, joint_id_to_art_id, art_joint_offsets, art_joint_counts);
                    Float m_ij = joint_joint_id_to_mass(I);
                    return m_ij;
                });

        namespace ERJ = sym::external_revolute_joint_constraint;

        // Compute Gradient
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                joint_id_to_G_theta.size(),
                [gradients = info.gradients().viewer().name("gradients"),
                 joint_id_to_G_theta = joint_id_to_G_theta.cviewer().name("G_theta"),
                 joint_id_to_body_ids = joint_id_to_body_ids.cviewer().name("joint_id_to_body_ids"),
                 joint_id_to_delta_theta = joint_id_to_delta_theta.cviewer().name("delta_theta"),
                 joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_id_to_art_id = joint_id_to_art_id.cviewer().name("joint_id_to_art_id"),
                 art_joint_offsets = art_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_joint_counts.cviewer().name("art_joint_counts"),
                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),
                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 qs = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs")] __device__(IndexT jointI) mutable
                {
                    Float    G_theta  = joint_id_to_G_theta(jointI);
                    Vector2i body_ids = joint_id_to_body_ids(jointI);
                    Vector6  basis_k  = joint_id_to_L_basis(jointI);
                    Vector6  basis_l  = joint_id_to_R_basis(jointI);

                    const Vector12& qk      = qs(body_ids[0]);
                    const Vector12& ql      = qs(body_ids[1]);
                    const Vector12& q_prevk = q_prevs(body_ids[0]);
                    const Vector12& q_prevl = q_prevs(body_ids[1]);

                    Vector12 F;
                    ERJ::F<Float>(F, basis_k, qk, basis_l, ql);
                    Vector12 F_t;
                    ERJ::F<Float>(F_t, basis_k, q_prevk, basis_l, q_prevl);

                    Vector12 dDeltaTheta_dF;
                    ERJ::dDeltaTheta_dF(dDeltaTheta_dF, F, F_t);

                    Vector<Float, 24> G24;
                    ERJ::JT_G<Float>(G24, dDeltaTheta_dF, basis_k, basis_l);

                    DoubletVectorAssembler DVA{gradients};

                    DVA.segment<2>(jointI * 2).write(body_ids, G24);
                });

        // Compute Hessian

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                joint_joint_id_to_mass.size(),
                [hessians = info.hessians().viewer().name("hessians"),
                 joint_id_to_G_theta = joint_id_to_G_theta.cviewer().name("joint_id_to_G_theta"),
                 joint_id_to_body_ids = joint_id_to_body_ids.cviewer().name("joint_id_to_body_ids"),
                 joint_id_to_delta_theta = joint_id_to_delta_theta.cviewer().name("delta_theta"),
                 joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_id_to_art_id = joint_id_to_art_id.cviewer().name("joint_id_to_art_id"),
                 art_joint_offsets = art_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_joint_counts.cviewer().name("art_joint_counts"),
                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),
                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 qs = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs")] __device__(IndexT joint_joint_I) mutable
                {
                    Float    m_ij = joint_joint_id_to_mass(joint_joint_I);
                    Vector2i ij   = joint_joint_id_to_ij(joint_joint_I,
                                                       joint_id_to_art_id,
                                                       art_joint_offsets,
                                                       art_joint_counts);


                    auto compute_F_F_t = [&](IndexT joint_id, Vector12& F, Vector12& F_t)
                    {
                        Vector6  basis_k  = joint_id_to_L_basis(joint_id);
                        Vector6  basis_l  = joint_id_to_R_basis(joint_id);
                        Vector2i body_ids = joint_id_to_body_ids(joint_id);

                        const Vector12& qk      = qs(body_ids[0]);
                        const Vector12& ql      = qs(body_ids[1]);
                        const Vector12& q_prevk = q_prevs(body_ids[0]);
                        const Vector12& q_prevl = q_prevs(body_ids[1]);

                        ERJ::F<Float>(F, basis_k, qk, basis_l, ql);
                        ERJ::F<Float>(F_t, basis_k, q_prevk, basis_l, q_prevl);
                    };

                    Vector12 Fi, Fi_t, Fj, Fj_t;

                    compute_F_F_t(ij[0], Fi, Fi_t);
                    compute_F_F_t(ij[1], Fj, Fj_t);


                    Vector12 dDeltaTheta_dFi, dDeltaTheta_dFj;
                    ERJ::dDeltaTheta_dF(dDeltaTheta_dFi, Fi, Fi_t);
                    ERJ::dDeltaTheta_dF(dDeltaTheta_dFj, Fj, Fj_t);

                    Matrix12x12 HF = dDeltaTheta_dFi * m_ij * dDeltaTheta_dFj.transpose();

                    if(ij[0] == ij[1])
                    {
                        auto        i         = ij[0];
                        Float       G_theta_i = joint_id_to_G_theta(i);
                        Matrix12x12 ddDeltaTheta_ddF;
                        ERJ::ddDeltaTheta_ddF(ddDeltaTheta_ddF, Fi, Fi_t);
                        HF += ddDeltaTheta_ddF;
                    }

                    make_spd(HF);

                    Matrix<Float, 24, 24> H24x24;

                    const Vector6& basis_k = joint_id_to_L_basis(ij[0]);
                    const Vector6& basis_l = joint_id_to_R_basis(ij[0]);
                    const Vector6& basis_m = joint_id_to_L_basis(ij[1]);
                    const Vector6& basis_n = joint_id_to_R_basis(ij[1]);

                    ERJ::JT_H_J<Float>(H24x24, HF, basis_k, basis_l, basis_m, basis_n);

                    Vector2i body_ids_i = joint_id_to_body_ids(ij[0]);
                    Vector2i body_ids_j = joint_id_to_body_ids(ij[1]);

                    TripletMatrixAssembler TMA{hessians};

                    TMA.block<2, 2>(joint_joint_I * 4).write(body_ids_i, body_ids_j, H24x24);
                });
    }
};
}  // namespace uipc::backend::cuda
