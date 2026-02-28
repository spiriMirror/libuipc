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
#include <time_integrator/time_integrator.h>
#include <kernel_cout.h>

// Ref: libuipc/scripts/symbol_calculation/external_articulation_constraint.ipynb

namespace uipc::backend::cuda
{
/**
 * @brief ExternalArticulationConstituion
 * 
 * Define an empty constitution for ExternalArticulationConstraint, 
 * because Constraint should be associated with a Constitution in UIPC design.
 * 
 * The actual constitutions are based on
 * - AffineBodyRevoluteJointConstitution (UID=18)
 * - AffineBodyPrismaticJointConstitution (UID=20)
 * 
 */
class ExternalArticulationConstituion final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 23ull;

    static constexpr U64 RevoluteJointConstitutionUID  = 18ull;
    static constexpr U64 PrismaticJointConstitutionUID = 20ull;

    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    void do_build(BuildInfo& info) override {}

    void do_init(FilteredInfo& info) override {}

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(0);
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        // no energy
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(0);
        info.hessian_count(0);
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        // no gradient and hessian
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};

REGISTER_SIM_SYSTEM(ExternalArticulationConstituion);

class ExternalArticulationConstraint final : public InterAffineBodyConstraint
{
  public:
    static constexpr U64   ConstraintUID   = 24ull;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;

    using InterAffineBodyConstraint::InterAffineBodyConstraint;

    vector<S<geometry::AttributeSlot<Float>>> h_art_joint_joint_mass;
    vector<S<geometry::AttributeSlot<Float>>> h_art_joint_delta_theta_tilde;

    OffsetCountCollection<IndexT> h_art_id_to_joint_offsets_counts;
    vector<IndexT>                h_joint_id_to_art_id;
    vector<U64>                   h_joint_id_to_uid;
    vector<Vector2i>              h_joint_id_to_body_ids;
    vector<Float>                 h_joint_id_to_delta_theta;
    vector<Float>                 h_joint_id_to_delta_theta_tilde;

    OffsetCountCollection<IndexT> h_art_id_to_joint_joint_offsets_counts;
    vector<IndexT>                h_joint_joint_id_to_art_id;
    vector<Vector2i>              h_joint_joint_id_to_joint_ij;
    vector<Float>                 h_joint_joint_id_to_mass;

    muda::DeviceBuffer<IndexT>   art_id_to_joint_offsets;
    muda::DeviceBuffer<IndexT>   art_id_to_joint_counts;
    muda::DeviceBuffer<IndexT>   joint_id_to_art_id;
    muda::DeviceBuffer<U64>      joint_id_to_uid;
    muda::DeviceBuffer<Vector2i> joint_id_to_body_ids;
    muda::DeviceBuffer<Float>    joint_id_to_delta_theta;
    muda::DeviceBuffer<Float>    joint_id_to_delta_theta_tilde;

    muda::DeviceBuffer<IndexT>   art_id_to_joint_joint_offsets;
    muda::DeviceBuffer<IndexT>   art_id_to_joint_joint_counts;
    muda::DeviceBuffer<Vector2i> joint_joint_id_to_joint_ij;
    muda::DeviceBuffer<Float>    joint_joint_id_to_mass;


    unordered_map<IndexT, IndexT> h_body_id_to_ref_q_prev_id;
    using AttrRefQPrev = S<const geometry::AttributeSlot<Vector12>>;
    vector<std::tuple<AttrRefQPrev, IndexT>> h_attr_ref_q_prevs;

    // joint_id -> [l_ref_q_prev_id, r_ref_q_prev_id]
    vector<Vector2i>             h_joint_id_to_ref_q_prev_ids;
    muda::DeviceBuffer<Vector2i> joint_id_to_ref_q_prev_ids;

    vector<Vector12>             h_ref_q_prevs;
    muda::DeviceBuffer<Vector12> ref_q_prevs;

    /**
     * Joint Basis:
     * Prismatic Joint: [c_bar, t_bar]
     * Revolute Joint:  [bn_bar, tn_bar]
     */

    vector<Vector6>             h_joint_id_to_L_basis;
    vector<Vector6>             h_joint_id_to_R_basis;
    muda::DeviceBuffer<Vector6> joint_id_to_L_basis;
    muda::DeviceBuffer<Vector6> joint_id_to_R_basis;

    // G^theta for each joint, intermediate variable for gradient and hessian computation
    muda::DeviceBuffer<Float> joint_id_to_G_theta;

    void do_build(BuildInfo& info) override
    {
        on_write_scene([this]() { write_scene(); });
    }

    U64 get_uid() const noexcept override { return ConstraintUID; }

    static auto get_revolute_basis(const geometry::SimplicialComplex* L,
                                   IndexT                             L_inst_id,
                                   const geometry::SimplicialComplex* R,
                                   IndexT                             R_inst_id,
                                   const geometry::SimplicialComplex* joint_mesh,
                                   IndexT joint_index)
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

    static auto get_prismatic_basis(const geometry::SimplicialComplex* L,
                                    IndexT L_inst_id,
                                    const geometry::SimplicialComplex* R,
                                    IndexT R_inst_id,
                                    const geometry::SimplicialComplex* joint_mesh,
                                    IndexT joint_index)
    {
        auto topo_view = joint_mesh->edges().topo().view();
        auto pos_view  = joint_mesh->positions().view();

        Vector2i e = topo_view[joint_index];
        Vector3  t = pos_view[e[1]] - pos_view[e[0]];
        t          = t.normalized();
        Vector3 c  = pos_view[e[0]];

        auto compute_ct_bar = [&](const geometry::SimplicialComplex* geo, IndexT inst_id) -> Vector6
        {
            UIPC_ASSERT(geo, "ExternalArticulationConstraint: Geometry is null when computing bn_bar");

            const Matrix4x4& trans = geo->transforms().view()[inst_id];
            Transform        T{trans};

            Matrix3x3 InvRot = T.rotation().inverse();
            Vector6   bn_bar;
            bn_bar.segment<3>(0) = T.inverse() * c;
            bn_bar.segment<3>(3) = InvRot * t;
            return bn_bar;
        };

        Vector6 L_ct_bar = compute_ct_bar(L, L_inst_id);
        Vector6 R_ct_bar = compute_ct_bar(R, R_inst_id);

        return std::tuple{L_ct_bar, R_ct_bar};
    }

    auto get_ref_q_prev(const geometry::SimplicialComplex* geo, IndexT inst_id, IndexT body_id)
    {
        IndexT ref_q_prev_id = -1;
        auto   ref_dof_prev  = geo->instances().find<Vector12>("ref_dof_prev");
        if(ref_dof_prev)
        {
            auto iter = h_body_id_to_ref_q_prev_id.find(body_id);
            if(iter != h_body_id_to_ref_q_prev_id.end())
            {
                ref_q_prev_id = iter->second;
            }
            else
            {
                auto ref_dof_prev_view = ref_dof_prev->view();
                ref_q_prev_id = static_cast<IndexT>(h_ref_q_prevs.size());
                // record it to use it in `do_step`
                h_attr_ref_q_prevs.push_back({ref_dof_prev, inst_id});
                h_ref_q_prevs.push_back(ref_dof_prev_view[inst_id]);
            }
        }

        return ref_q_prev_id;
    }

    void build_joint_info(span<S<geometry::GeometrySlot>>        geo_slots,
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
        UIPC_ASSERT(uid_value == ExternalArticulationConstituion::RevoluteJointConstitutionUID
                        || uid_value == ExternalArticulationConstituion::PrismaticJointConstitutionUID,
                    "ExternalArticulationConstraint: Now only support RevoluteJointConstraint (UID={}) and PrismaticJointConstraint (UID={}), but found joint with UID {}",
                    ExternalArticulationConstituion::RevoluteJointConstitutionUID,
                    ExternalArticulationConstituion::PrismaticJointConstitutionUID,
                    uid_value);

        h_joint_id_to_uid.push_back(uid_value);

        // 1. Get Joint's two Links
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
        h_joint_id_to_body_ids.push_back(body_id);

        auto L_link_geo = info.body_geo(geo_slots, geo_id[0]);
        auto R_link_geo = info.body_geo(geo_slots, geo_id[1]);

        Vector6 L_basis, R_basis;

        if(uid_value == ExternalArticulationConstituion::RevoluteJointConstitutionUID)
        {
            std::tie(L_basis, R_basis) = get_revolute_basis(
                L_link_geo, inst_id[0], R_link_geo, inst_id[1], joint_mesh, index);
        }
        else if(uid_value == ExternalArticulationConstituion::PrismaticJointConstitutionUID)
        {
            std::tie(L_basis, R_basis) = get_prismatic_basis(
                L_link_geo, inst_id[0], R_link_geo, inst_id[1], joint_mesh, index);
        }

        h_joint_id_to_L_basis.push_back(L_basis);
        h_joint_id_to_R_basis.push_back(R_basis);

        // 2. Get Joint's 2 ref_q_prev (if any)
        Vector2i ref_q_prev_ids = {
            get_ref_q_prev(L_link_geo, inst_id[0], body_id[0]),
            get_ref_q_prev(R_link_geo, inst_id[1], body_id[1]),
        };

        h_joint_id_to_ref_q_prev_ids.push_back(ref_q_prev_ids);
    }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto scene     = world().scene();
        auto geo_slots = scene.geometries();

        auto art_count = info.anim_inter_geo_infos().size();

        h_art_id_to_joint_offsets_counts.resize(art_count);
        h_art_id_to_joint_joint_offsets_counts.resize(art_count);

        // 1. Collect Data
        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto joint_collection = geo["joint"];
                UIPC_ASSERT(joint_collection, "ExternalArticulationConstraint: 'joint' attribute collection not found in geometry");

                h_art_id_to_joint_offsets_counts.counts()[I.index()] =
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
                            "ExternalArticulationConstraint: 'joint_joint' collection size mismatch, expected {} * {} = {}, got {}",
                            joint_collection->size(),
                            joint_collection->size(),
                            joint_collection->size() * joint_collection->size(),
                            joint_joint_collection->size());

                auto mass = joint_joint_collection->find<Float>("mass");
                UIPC_ASSERT(mass, "ExternalArticulationConstraint: 'mass' attribute not found in joint_joint collection");

                auto mass_view = mass->view();

                h_art_id_to_joint_joint_offsets_counts.counts()[I.index()] =
                    mass->size();

                for(auto&& [geo_id, index, delta_theta_tilde] :
                    zip(geo_id_view, index_view, delta_theta_tilde_view))
                {
                    // find the joint geometry in the scene
                    auto geo_slot = scene.find_geometry(geo_id);

                    h_joint_id_to_art_id.push_back(I.index());
                    h_joint_id_to_delta_theta_tilde.push_back(delta_theta_tilde);

                    build_joint_info(geo_slots, info, geo_slot->geometry(), index);
                }

                for(auto&& m : mass_view)
                {
                    h_joint_joint_id_to_mass.push_back(m);
                    h_joint_joint_id_to_art_id.push_back(I.index());
                }

                auto n_joint = joint_collection->size();
                for(IndexT i = 0; i < n_joint; ++i)
                {
                    for(IndexT j = 0; j < n_joint; ++j)
                    {
                        // i must go slower than j
                        // so that the `fast_segmental_reduce` can work correctly in gradient/hessian computation
                        h_joint_joint_id_to_joint_ij.push_back(Vector2i{i, j});
                    }
                }

                // theta_tilde and mass will change in each step, so we just prepare the slots here
                // later we will fill them in do_step()
                h_art_joint_delta_theta_tilde.push_back(delta_theta_tilde);
                h_art_joint_joint_mass.push_back(mass);
            });

        // 2. Compute Offsets
        h_art_id_to_joint_offsets_counts.scan();
        // shift the joint_ij by joint offsets
        for(auto&& [I, ij] : enumerate(h_joint_joint_id_to_joint_ij))
        {
            auto art_id   = h_joint_joint_id_to_art_id[I];
            auto j_offset = h_art_id_to_joint_offsets_counts.offsets()[art_id];
            ij += Vector2i{j_offset, j_offset};
        }
        h_art_id_to_joint_joint_offsets_counts.scan();

        // 3. Upload to Device

        // Joint Info
        auto n_joint = h_joint_id_to_art_id.size();

        auto h_art_joint_offsets = h_art_id_to_joint_offsets_counts.offsets();
        art_id_to_joint_offsets.resize(h_art_joint_offsets.size());
        art_id_to_joint_offsets.view().copy_from(h_art_joint_offsets.data());

        auto h_art_joint_counts = h_art_id_to_joint_offsets_counts.counts();
        art_id_to_joint_counts.resize(h_art_joint_counts.size());
        art_id_to_joint_counts.view().copy_from(h_art_joint_counts.data());

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
        auto h_art_mass_offsets = h_art_id_to_joint_joint_offsets_counts.offsets();
        art_id_to_joint_joint_offsets.resize(h_art_mass_offsets.size());
        art_id_to_joint_joint_offsets.view().copy_from(h_art_mass_offsets.data());

        auto h_art_mass_counts = h_art_id_to_joint_joint_offsets_counts.counts();
        art_id_to_joint_joint_counts.resize(h_art_mass_counts.size());
        art_id_to_joint_joint_counts.view().copy_from(h_art_mass_counts.data());

        joint_joint_id_to_mass.resize(h_joint_joint_id_to_mass.size());
        joint_joint_id_to_mass.view().copy_from(h_joint_joint_id_to_mass.data());

        joint_joint_id_to_joint_ij.resize(h_joint_joint_id_to_joint_ij.size());
        joint_joint_id_to_joint_ij.view().copy_from(h_joint_joint_id_to_joint_ij.data());

        // Ref q_prev
        joint_id_to_ref_q_prev_ids.resize(h_joint_id_to_ref_q_prev_ids.size());
        joint_id_to_ref_q_prev_ids.view().copy_from(h_joint_id_to_ref_q_prev_ids.data());

        ref_q_prevs.resize(h_ref_q_prevs.size());
        ref_q_prevs.view().copy_from(h_ref_q_prevs.data());
    }

    void do_step(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        // only collect mass and delta_theta_tilde
        auto geo_slots = world().scene().geometries();

        // Update Joint Mass
        h_joint_joint_id_to_mass.clear();
        for(auto&& mass : h_art_joint_joint_mass)
        {
            auto mass_view = mass->view();
            std::ranges::copy(mass_view, std::back_inserter(h_joint_joint_id_to_mass));
        }

        // Update Predicted Delta Theta
        h_joint_id_to_delta_theta_tilde.clear();
        for(auto&& delta_theta_tilde : h_art_joint_delta_theta_tilde)
        {
            auto delta_theta_tilde_view = delta_theta_tilde->view();
            std::ranges::copy(delta_theta_tilde_view,
                              std::back_inserter(h_joint_id_to_delta_theta_tilde));
        }

        // Update Ref Dof Prev
        std::ranges::transform(h_attr_ref_q_prevs,
                               h_ref_q_prevs.begin(),
                               [&](const std::tuple<AttrRefQPrev, IndexT>& tuple) -> Vector12
                               {
                                   auto&& [attr, inst_id] = tuple;
                                   auto view              = attr->view();
                                   return view[inst_id];
                               });

        // upload to device
        auto async_copy = []<typename T>(muda::DeviceBuffer<T>& device_buffer,
                                         span<const T>          host_vector)
        {
            muda::BufferLaunch().copy<T>(device_buffer.view(), host_vector.data());
        };

        async_copy(joint_joint_id_to_mass, span<const Float>{h_joint_joint_id_to_mass});
        async_copy(joint_id_to_delta_theta_tilde,
                   span<const Float>{h_joint_id_to_delta_theta_tilde});
        async_copy(ref_q_prevs, span<const Vector12>{h_ref_q_prevs});
    }

    void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override
    {
        auto e_count = joint_joint_id_to_mass.size();
        info.energy_count(e_count);
        auto g_count = joint_id_to_art_id.size() * 2;
        info.gradient_count(g_count);

        if(info.gradient_only())
            return;

        auto h_count = joint_joint_id_to_mass.size() * HalfHessianSize;
        info.hessian_count(h_count);
    }

    void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) override
    {
        using namespace muda;

        namespace ERJ = sym::external_revolute_joint_constraint;
        namespace EPJ = sym::external_prismatic_joint_constraint;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                joint_joint_id_to_mass.size(),
                [energies = info.energies().viewer().name("energies"),
                 joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_joint_id_to_joint_ij =
                     joint_joint_id_to_joint_ij.cviewer().name("joint_joint_id_to_joint_ij"),

                 art_joint_joint_offsets =
                     art_id_to_joint_joint_offsets.cviewer().name("art_joint_joint_offsets"),
                 art_joint_joint_counts = art_id_to_joint_joint_counts.cviewer().name("art_joint_joint_counts"),
                 art_joint_offsets = art_id_to_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_id_to_joint_counts.cviewer().name("art_joint_counts"),

                 joint_id_to_delta_theta = joint_id_to_delta_theta.cviewer().name("joint_id_to_delta_theta"),
                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),
                 joint_id_to_body_ids = joint_id_to_body_ids.cviewer().name("joint_id_to_body_ids"),
                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 joint_id_to_uid = joint_id_to_uid.cviewer().name("joint_id_to_uid"),

                 qs      = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs"),

                 joint_id_to_ref_q_prev_ids =
                     joint_id_to_ref_q_prev_ids.cviewer().name("joint_id_to_ref_q_prev_ids"),
                 ref_q_prevs = ref_q_prevs.viewer().name("ref_q_prevs"),

                 PrismaticUID = ExternalArticulationConstituion::PrismaticJointConstitutionUID,
                 RevoluteUID =
                     ExternalArticulationConstituion::RevoluteJointConstitutionUID] __device__(IndexT I) mutable
                {
                    Float    m_ij = joint_joint_id_to_mass(I);
                    Vector2i ij   = joint_joint_id_to_joint_ij(I);

                    auto compute_delta_theta = [&](IndexT joint_id) -> Float
                    {
                        Vector6  basis_k  = joint_id_to_L_basis(joint_id);
                        Vector6  basis_l  = joint_id_to_R_basis(joint_id);
                        Vector2i body_ids = joint_id_to_body_ids(joint_id);

                        const Vector12& qk = qs(body_ids[0]);
                        const Vector12& ql = qs(body_ids[1]);

                        const Vector2i& ref_q_prev_ids =
                            joint_id_to_ref_q_prev_ids(joint_id);

                        const Vector12& q_prevk = ref_q_prev_ids[0] >= 0 ?
                                                      ref_q_prevs(ref_q_prev_ids[0]) :
                                                      q_prevs(body_ids[0]);

                        const Vector12& q_prevl = ref_q_prev_ids[1] >= 0 ?
                                                      ref_q_prevs(ref_q_prev_ids[1]) :
                                                      q_prevs(body_ids[1]);

                        U64 joint_uid = joint_id_to_uid(joint_id);

                        Float delta_theta;

                        if(joint_uid == RevoluteUID)
                        {
                            ERJ::DeltaTheta(delta_theta, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                        }
                        else if(joint_uid == PrismaticUID)
                        {
                            EPJ::DeltaTheta(delta_theta, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                        }
                        else
                        {
                            MUDA_ASSERT(false,
                                        "ExternalArticulationConstraint: Unknown joint UID {}",
                                        joint_uid);
                            // should not reach here
                            delta_theta = 0.0f;
                        }


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
        namespace ERJ = sym::external_revolute_joint_constraint;
        namespace EPJ = sym::external_prismatic_joint_constraint;

        // Compute G^theta
        FastSegmentalReduce()
            .file_line(__FILE__, __LINE__)
            .reduce(
                joint_joint_id_to_mass.size(),
                joint_id_to_G_theta.view(),
                [joint_joint_id_to_joint_ij =
                     joint_joint_id_to_joint_ij.cviewer().name(
                         "joint_joint_id_to_joint_ij")] __device__(IndexT I) -> IndexT
                {
                    Vector2i ij = joint_joint_id_to_joint_ij(I);
                    return ij[0];
                },
                [joint_joint_id_to_mass = joint_joint_id_to_mass.cviewer().name("masses"),
                 joint_joint_id_to_joint_ij =
                     joint_joint_id_to_joint_ij.cviewer().name("joint_joint_id_to_joint_ij"),
                 art_joint_joint_offsets =
                     art_id_to_joint_joint_offsets.cviewer().name("art_joint_joint_offsets"),
                 art_joint_joint_counts = art_id_to_joint_joint_counts.cviewer().name("art_joint_joint_counts"),
                 art_joint_offsets = art_id_to_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_id_to_joint_counts.cviewer().name("art_joint_counts"),
                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 joint_id_to_body_ids = joint_id_to_body_ids.cviewer().name("joint_id_to_body_ids"),
                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),

                 qs      = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs"),
                 joint_id_to_ref_q_prev_ids =
                     joint_id_to_ref_q_prev_ids.cviewer().name("joint_id_to_ref_q_prev_ids"),
                 ref_q_prevs = ref_q_prevs.viewer().name("ref_q_prevs"),

                 joint_id_to_uid = joint_id_to_uid.cviewer().name("joint_id_to_uid"),
                 RevoluteUID = ExternalArticulationConstituion::RevoluteJointConstitutionUID,
                 PrismaticUID =
                     ExternalArticulationConstituion::PrismaticJointConstitutionUID] __device__(IndexT joint_jointI) -> Float
                {
                    Vector2i ij   = joint_joint_id_to_joint_ij(joint_jointI);
                    Float    m_ij = joint_joint_id_to_mass(joint_jointI);

                    auto joint_id = ij[1];

                    Vector6  basis_k  = joint_id_to_L_basis(joint_id);
                    Vector6  basis_l  = joint_id_to_R_basis(joint_id);
                    Vector2i body_ids = joint_id_to_body_ids(joint_id);

                    const Vector12& qk = qs(body_ids[0]);
                    const Vector12& ql = qs(body_ids[1]);

                    const Vector2i& ref_q_prev_ids = joint_id_to_ref_q_prev_ids(joint_id);

                    const Vector12& q_prevk = ref_q_prev_ids[0] >= 0 ?
                                                  ref_q_prevs(ref_q_prev_ids[0]) :
                                                  q_prevs(body_ids[0]);

                    const Vector12& q_prevl = ref_q_prev_ids[1] >= 0 ?
                                                  ref_q_prevs(ref_q_prev_ids[1]) :
                                                  q_prevs(body_ids[1]);

                    U64 joint_uid = joint_id_to_uid(joint_id);

                    Float delta_theta_j;

                    if(joint_uid == RevoluteUID)
                    {
                        ERJ::DeltaTheta<Float>(
                            delta_theta_j, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                    }
                    else if(joint_uid == PrismaticUID)
                    {
                        EPJ::DeltaTheta<Float>(
                            delta_theta_j, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                    }
                    else
                    {
                        // should not reach here
                        delta_theta_j = 0.0f;
                    }

                    Float delta_theta_tilde_j = joint_id_to_delta_theta_tilde(joint_id);

                    return m_ij * (delta_theta_j - delta_theta_tilde_j);
                });


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
                 art_joint_offsets = art_id_to_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_id_to_joint_counts.cviewer().name("art_joint_counts"),
                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),
                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 qs      = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs"),
                 joint_id_to_ref_q_prev_ids =
                     joint_id_to_ref_q_prev_ids.cviewer().name("joint_id_to_ref_q_prev_ids"),
                 ref_q_prevs = ref_q_prevs.viewer().name("ref_q_prevs"),

                 joint_id_to_uid = joint_id_to_uid.cviewer().name("joint_id_to_uid"),
                 RevoluteUID = ExternalArticulationConstituion::RevoluteJointConstitutionUID,
                 PrismaticUID =
                     ExternalArticulationConstituion::PrismaticJointConstitutionUID] __device__(IndexT jointI) mutable
                {
                    Float    G_theta  = joint_id_to_G_theta(jointI);
                    Vector2i body_ids = joint_id_to_body_ids(jointI);
                    Vector6  basis_k  = joint_id_to_L_basis(jointI);
                    Vector6  basis_l  = joint_id_to_R_basis(jointI);

                    const Vector12& qk = qs(body_ids[0]);
                    const Vector12& ql = qs(body_ids[1]);

                    const Vector2i& ref_q_prev_ids = joint_id_to_ref_q_prev_ids(jointI);

                    const Vector12& q_prevk = ref_q_prev_ids[0] >= 0 ?
                                                  ref_q_prevs(ref_q_prev_ids[0]) :
                                                  q_prevs(body_ids[0]);

                    const Vector12& q_prevl = ref_q_prev_ids[1] >= 0 ?
                                                  ref_q_prevs(ref_q_prev_ids[1]) :
                                                  q_prevs(body_ids[1]);

                    auto joint_uid = joint_id_to_uid(jointI);

                    Vector<Float, 24> G24;
                    if(joint_uid == RevoluteUID)
                    {
                        ERJ::dDeltaTheta_dQ(G24, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                    }
                    else if(joint_uid == PrismaticUID)
                    {
                        EPJ::dDeltaTheta_dQ(G24, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                    }
                    else
                    {
                        MUDA_ASSERT(false,
                                    "ExternalArticulationConstraint: Unknown joint UID {}",
                                    joint_uid);
                        G24.setZero();
                    }

                    G24 *= G_theta;

                    DoubletVectorAssembler DVA{gradients};

                    DVA.segment<2>(jointI * 2).write(body_ids, G24);
                });

        if(info.gradient_only())
            return;

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
                 joint_joint_id_to_joint_ij =
                     joint_joint_id_to_joint_ij.cviewer().name("joint_joint_id_to_joint_ij"),

                 art_joint_joint_offsets =
                     art_id_to_joint_joint_offsets.cviewer().name("art_joint_joint_offsets"),
                 art_joint_joint_counts = art_id_to_joint_joint_counts.cviewer().name("art_joint_joint_counts"),
                 art_joint_offsets = art_id_to_joint_offsets.cviewer().name("art_joint_offsets"),
                 art_joint_counts = art_id_to_joint_counts.cviewer().name("art_joint_counts"),

                 joint_id_to_delta_theta_tilde =
                     joint_id_to_delta_theta_tilde.cviewer().name("joint_id_to_delta_theta_tilde"),
                 joint_id_to_L_basis = joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 qs      = info.qs().viewer().name("qs"),
                 q_prevs = info.q_prevs().viewer().name("q_prevs"),
                 joint_id_to_ref_q_prev_ids =
                     joint_id_to_ref_q_prev_ids.cviewer().name("joint_id_to_ref_q_prev_ids"),
                 ref_q_prevs = ref_q_prevs.viewer().name("ref_q_prevs"),
                 joint_id_to_uid = joint_id_to_uid.cviewer().name("joint_id_to_uid"),
                 RevoluteUID = ExternalArticulationConstituion::RevoluteJointConstitutionUID,
                 PrismaticUID =
                     ExternalArticulationConstituion::PrismaticJointConstitutionUID] __device__(IndexT joint_joint_I) mutable
                {
                    Float    m_ij = joint_joint_id_to_mass(joint_joint_I);
                    Vector2i ij   = joint_joint_id_to_joint_ij(joint_joint_I);

                    auto compute_dDeltaTheta_dQ = [&](IndexT joint_id) -> Vector<Float, 24>
                    {
                        U64 joint_uid = joint_id_to_uid(joint_id);

                        Vector6  basis_k   = joint_id_to_L_basis(joint_id);
                        Vector6  basis_l   = joint_id_to_R_basis(joint_id);
                        Vector2i body_ids  = joint_id_to_body_ids(joint_id);
                        const Vector12& qk = qs(body_ids[0]);
                        const Vector12& ql = qs(body_ids[1]);
                        const Vector2i& ref_q_prev_ids =
                            joint_id_to_ref_q_prev_ids(joint_id);
                        const Vector12& q_prevk = ref_q_prev_ids[0] >= 0 ?
                                                      ref_q_prevs(ref_q_prev_ids[0]) :
                                                      q_prevs(body_ids[0]);
                        const Vector12& q_prevl = ref_q_prev_ids[1] >= 0 ?
                                                      ref_q_prevs(ref_q_prev_ids[1]) :
                                                      q_prevs(body_ids[1]);

                        Vector<Float, 24> dDeltaTheta_dQ;

                        U64 uid = joint_id_to_uid(joint_id);
                        if(uid == RevoluteUID)
                        {
                            ERJ::dDeltaTheta_dQ(
                                dDeltaTheta_dQ, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                        }
                        else if(uid == PrismaticUID)
                        {
                            EPJ::dDeltaTheta_dQ(
                                dDeltaTheta_dQ, basis_k, qk, q_prevk, basis_l, ql, q_prevl);
                        }
                        else
                        {
                            MUDA_ASSERT(false, "ExternalArticulationConstraint: Unknown joint UID {}", uid);
                            dDeltaTheta_dQ.setZero();
                        }
                        return dDeltaTheta_dQ;
                    };

                    Vector<Float, 24> dDeltaTheta_dQ_i = compute_dDeltaTheta_dQ(ij[0]);
                    Vector<Float, 24> dDeltaTheta_dQ_j = compute_dDeltaTheta_dQ(ij[1]);


                    Matrix<Float, 24, 24> H24x24 =
                        dDeltaTheta_dQ_i * m_ij * dDeltaTheta_dQ_j.transpose();

                    if(ij[0] == ij[1])
                    {
                        auto i = ij[0];

                        U64 joint_uid = joint_id_to_uid(i);

                        Vector6  basis_k  = joint_id_to_L_basis(i);
                        Vector6  basis_l  = joint_id_to_R_basis(i);
                        Vector2i body_ids = joint_id_to_body_ids(i);

                        const Vector12& qk = qs(body_ids[0]);
                        const Vector12& ql = qs(body_ids[1]);

                        const Vector2i& ref_q_prev_ids = joint_id_to_ref_q_prev_ids(i);

                        const Vector12& q_prevk = ref_q_prev_ids[0] >= 0 ?
                                                      ref_q_prevs(ref_q_prev_ids[0]) :
                                                      q_prevs(body_ids[0]);

                        const Vector12& q_prevl = ref_q_prev_ids[1] >= 0 ?
                                                      ref_q_prevs(ref_q_prev_ids[1]) :
                                                      q_prevs(body_ids[1]);

                        Float G_theta_i = joint_id_to_G_theta(i);

                        Vector<Float, 12>     F, F_t;
                        Matrix<Float, 24, 24> JT_H_J;
                        if(joint_uid == RevoluteUID)
                        {
                            ERJ::F<Float>(F, basis_k, qk, basis_l, ql);
                            ERJ::F<Float>(F_t, basis_k, q_prevk, basis_l, q_prevl);
                            Matrix12x12 ddDeltaTheta_ddF;
                            ERJ::ddDeltaTheta_ddF(ddDeltaTheta_ddF, F, F_t);
                            Matrix12x12 HF = G_theta_i * ddDeltaTheta_ddF;
                            make_spd(HF);
                            ERJ::JT_H_J(JT_H_J, HF, basis_k, basis_l, basis_k, basis_l);
                        }
                        else if(joint_uid == PrismaticUID)
                        {
                            EPJ::F<Float>(F, basis_k, qk, basis_l, ql);
                            EPJ::F<Float>(F_t, basis_k, q_prevk, basis_l, q_prevl);
                            Matrix12x12 ddDeltaTheta_ddF;
                            EPJ::ddDeltaTheta_ddF(ddDeltaTheta_ddF, F, F_t);
                            Matrix12x12 HF = G_theta_i * ddDeltaTheta_ddF;
                            make_spd(HF);
                            EPJ::JT_H_J(JT_H_J, HF, basis_k, basis_l, basis_k, basis_l);
                        }
                        else
                        {
                            MUDA_ASSERT(false,
                                        "ExternalArticulationConstraint: Unknown joint UID {}",
                                        joint_uid);
                        }

                        H24x24 += JT_H_J;
                    }

                    Vector2i body_ids_i = joint_id_to_body_ids(ij[0]);
                    Vector2i body_ids_j = joint_id_to_body_ids(ij[1]);

                    TripletMatrixAssembler TMA{hessians};

                    TMA.half_block<2>(joint_joint_I * HalfHessianSize).write(body_ids_i, body_ids_j, H24x24);
                });
    }

    void write_scene()
    {
        auto geo_slots = world().scene().geometries();

        joint_id_to_delta_theta.copy_to(h_joint_id_to_delta_theta);

        this->for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I_info,
                geometry::Geometry&                                    geo)
            {
                auto joint_collection = geo["joint"];
                auto delta_theta = joint_collection->find<Float>("delta_theta");

                if(delta_theta)
                {
                    auto delta_theta_view = view(*delta_theta);
                    auto [offset, count] =
                        h_art_id_to_joint_offsets_counts[I_info.index()];
                    std::ranges::copy(span{h_joint_id_to_delta_theta}.subspan(offset, count),
                                      delta_theta_view.begin());
                }
            });
    }

    BufferDump dump_delta_theta;

    bool do_dump(DumpInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return dump_delta_theta.dump(fmt::format("{}delta_theta.{}", path, frame),
                                     joint_id_to_delta_theta);
    }

    bool do_try_recover(RecoverInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return dump_delta_theta.load(fmt::format("{}delta_theta.{}", path, frame));
    }

    void do_apply_recover(RecoverInfo& info) override
    {
        dump_delta_theta.apply_to(joint_id_to_delta_theta);
    }

    void do_clear_recover(RecoverInfo& info) override
    {
        dump_delta_theta.clean_up();
    }
};

REGISTER_SIM_SYSTEM(ExternalArticulationConstraint);

class ExternalArticulationConstraintTimeIntegrator final : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<ExternalArticulationConstraint> constraint;
    SimSystemSlot<AffineBodyDynamics>             affine_body_dynamics;

    void do_build(BuildInfo& info) override
    {
        constraint           = require<ExternalArticulationConstraint>();
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_init(InitInfo& info) override {}

    void do_predict_dof(PredictDofInfo& info) override
    {
        // No thing
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace ERJ = sym::external_revolute_joint_constraint;
        namespace EPJ = sym::external_prismatic_joint_constraint;

        // Compute delta_theta
        auto& c   = constraint;
        auto& abd = affine_body_dynamics;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                c->joint_id_to_delta_theta.size(),
                [joint_id_to_delta_theta = c->joint_id_to_delta_theta.viewer().name("joint_id_to_delta_theta"),
                 joint_id_to_body_ids = c->joint_id_to_body_ids.cviewer().name("joint_id_to_body_ids"),
                 joint_id_to_L_basis = c->joint_id_to_L_basis.cviewer().name("joint_id_to_L_basis"),
                 joint_id_to_R_basis = c->joint_id_to_R_basis.cviewer().name("joint_id_to_R_basis"),
                 qs      = abd->qs().viewer().name("qs"),
                 q_prevs = abd->q_prevs().viewer().name("q_prevs"),
                 joint_id_to_ref_q_prev_ids =
                     c->joint_id_to_ref_q_prev_ids.cviewer().name("joint_id_to_ref_q_prev_ids"),
                 ref_q_prevs = c->ref_q_prevs.viewer().name("ref_q_prevs"),
                 joint_id_to_uid = c->joint_id_to_uid.cviewer().name("joint_id_to_uid"),
                 RevoluteUID = ExternalArticulationConstituion::RevoluteJointConstitutionUID,
                 PrismaticUID =
                     ExternalArticulationConstituion::PrismaticJointConstitutionUID] __device__(IndexT jointI) mutable
                {
                    Vector6  L_basis  = joint_id_to_L_basis(jointI);
                    Vector6  R_basis  = joint_id_to_R_basis(jointI);
                    Vector2i body_ids = joint_id_to_body_ids(jointI);

                    const Vector12& qk = qs(body_ids[0]);
                    const Vector12& ql = qs(body_ids[1]);

                    const Vector2i& ref_q_prev_ids = joint_id_to_ref_q_prev_ids(jointI);

                    const Vector12& q_prevk = ref_q_prev_ids[0] >= 0 ?
                                                  ref_q_prevs(ref_q_prev_ids[0]) :
                                                  q_prevs(body_ids[0]);

                    const Vector12& q_prevl = ref_q_prev_ids[1] >= 0 ?
                                                  ref_q_prevs(ref_q_prev_ids[1]) :
                                                  q_prevs(body_ids[1]);

                    auto  joint_uid = joint_id_to_uid(jointI);
                    Float delta_theta;
                    if(joint_uid == RevoluteUID)
                    {
                        ERJ::DeltaTheta<Float>(
                            delta_theta, L_basis, qk, q_prevk, R_basis, ql, q_prevl);
                    }
                    else if(joint_uid == PrismaticUID)
                    {
                        EPJ::DeltaTheta<Float>(
                            delta_theta, L_basis, qk, q_prevk, R_basis, ql, q_prevl);
                    }
                    else
                    {
                        MUDA_ASSERT(false,
                                    "ExternalArticulationConstraint: Unknown joint UID {}",
                                    joint_uid);
                        // should not reach here
                        delta_theta = 0.0f;
                    }

                    joint_id_to_delta_theta(jointI) = delta_theta;
                });
    }
};

REGISTER_SIM_SYSTEM(ExternalArticulationConstraintTimeIntegrator);
}  // namespace uipc::backend::cuda
