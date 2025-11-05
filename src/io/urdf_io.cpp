#include <urdf_parser/urdf_parser.h>

#include <uipc/io/urdf_io.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/geometry/simplicial_complex_slot.h>
#include <uipc/geometry/implicit_geometry_slot.h>
#include <uipc/geometry/implicit_geometry.h>
#include <filesystem>
#include <Eigen/Geometry>
#include <uipc/common/magic_enum.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/zip.h>
#include <uipc/geometry/utils/label_surface.h>

#define URDFIO_INFO "[UrdfIO]"

namespace uipc::io
{
namespace fs = std::filesystem;

// -----------------------------------------------------------------------------
// Basis conversions
// URDF:  +x forward, +y left, +z up
// UIPC:  +x right,   +y up,   -z forward
// -----------------------------------------------------------------------------

static Matrix4x4 basis_urdf_to_uipc()
{
    Matrix4x4 M = Matrix4x4::Identity();
    // rows of rotation part
    M.block<1, 3>(0, 0) = Vector3{0, -1, 0};
    M.block<1, 3>(1, 0) = Vector3{0, 0, 1};
    M.block<1, 3>(2, 0) = Vector3{-1, 0, 0};
    return M;
}

//static Matrix4x4 basis_uipc_to_urdf()
//{
//    Matrix4x4 M         = Matrix4x4::Identity();
//    M.block<1, 3>(0, 0) = Vector3{0, 0, -1};
//    M.block<1, 3>(1, 0) = Vector3{-1, 0, 0};
//    M.block<1, 3>(2, 0) = Vector3{0, 1, 0};
//    return M;
//}

static Matrix4x4 to_affine(const urdf::Pose& pose)
{
    Transform T = Transform::Identity();
    T.translate(Vector3{pose.position.x, pose.position.y, pose.position.z});
    const Eigen::Quaterniond q{
        pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z};
    T.rotate(q);
    return T.matrix();
}

class JointInfo
{
  public:
    std::string name;
    std::string parent_link_name;
    std::string child_link_name;

    Matrix4x4 local_trans = Matrix4x4::Identity();  // local transform of the joint
    Matrix4x4 current_local_trans = Matrix4x4::Identity();  // current local transform of the joint
    Matrix4x4 global_trans = Matrix4x4::Identity();  // global transform of the joint
};

class RevoluteJointInfo
{
  public:
    Vector3 local_axis;  // axis of rotation
    Vector2 limits;      // [min, max]

    std::string mimic_joint_name;                       // mimic joint name
    Vector2 mimic_multiplier_offset = Vector2::Zero();  // mimic joint multiplier offset

    Float angle = 0.0;  // current angle of the joint
};

class FixedJointInfo
{
  public:
};

class UrdfController::Impl
{
  public:
    urdf::ModelInterfaceSharedPtr model;
    Matrix4x4 root_link_transform = Matrix4x4::Identity();  // root link transform
    std::map<std::string, IndexT> link_geo_id_map;
    std::map<std::string, S<geometry::SimplicialComplexSlot>> link_vis_geo_slots;
    std::map<std::string, S<geometry::SimplicialComplexSlot>> link_geo_slots;
    std::map<std::string, JointInfo>                          joint_infos;
    std::map<std::string, FixedJointInfo>                     fixed_joint_infos;
    std::map<std::string, RevoluteJointInfo> revolute_joint_infos;
    S<geometry::ImplicitGeometrySlot>        revolute_joints;
    std::filesystem::path                    urdf_folder;

    void move_root(const Vector3& trans, const Vector3& rot)
    {
        Transform t = Transform::Identity();
        t.translate(trans);
        t.rotate(Eigen::AngleAxisd{rot[0], Vector3::UnitX()});
        t.rotate(Eigen::AngleAxisd{rot[1], Vector3::UnitY()});
        t.rotate(Eigen::AngleAxisd{rot[2], Vector3::UnitZ()});
        root_link_transform = t.matrix();
    }

    // Propagate the transform from the root link to all child links
    void propagate_transform()
    {
        auto& root_link = model->root_link_;
        UIPC_ASSERT(root_link != nullptr, "Root link is null, why?");
        _propagate_transform(*root_link, root_link_transform);
    }

    void _propagate_transform(urdf::Link& link, const Matrix4x4& p)
    {
        for(auto&& [joint, link] : zip(link.child_joints, link.child_links))
        {
            auto& joint_info        = joint_infos[joint->name];
            joint_info.global_trans = p * joint_info.current_local_trans;
            _propagate_transform(*link, joint_info.global_trans);
        }
    }

    void rotate_to(std::string_view joint_name, Float angle)
    {
        auto it = revolute_joint_infos.find(std::string{joint_name});
        UIPC_ASSERT(it != revolute_joint_infos.end(),
                    "Joint {} not found in `Revolute Joints`, can't rotate",
                    joint_name);

        if(!it->second.mimic_joint_name.empty())
            return;

        it->second.angle = angle;
    }

    void apply_rotation()
    {
        for(auto&& [name, r_joint] : revolute_joint_infos)
        {
            auto&   joint_info = joint_infos[name];
            Float   angle      = r_joint.angle;
            Vector3 axis       = r_joint.local_axis;

            Eigen::AngleAxisd aa{angle, axis};
            Transform         t = Transform::Identity();
            t.matrix()          = joint_info.local_trans;
            t.rotate(aa);
            joint_info.current_local_trans = t.matrix();
        }
    }

    void process_mimic()
    {
        for(auto&& [name, r_joint] : revolute_joint_infos)
        {
            auto& joint_info = joint_infos[name];
            if(r_joint.mimic_joint_name.empty())
                continue;

            auto it = revolute_joint_infos.find(r_joint.mimic_joint_name);
            UIPC_ASSERT(it != revolute_joint_infos.end(),
                        "Mimic joint {} not found, can't process mimic, why can it happen?",
                        r_joint.mimic_joint_name);

            auto& mimic_joint_info = it->second;
            r_joint.angle = mimic_joint_info.angle * r_joint.mimic_multiplier_offset[0]
                            + r_joint.mimic_multiplier_offset[1];
        }
    }

    void apply_to(std::string_view attr)
    {
        auto URDF2UIPC = basis_urdf_to_uipc();

        process_mimic();

        apply_rotation();

        propagate_transform();

        // process the root link
        {
            auto root_name = model->root_link_->name;
            auto it        = link_geo_slots.find(root_name);
            if(it != link_geo_slots.end())
            {
                auto& slot = it->second;
                auto trans = slot->geometry().instances().find<Matrix4x4>(attr);
                auto trans_view = view(*trans);
                trans_view[0]   = URDF2UIPC * root_link_transform;
            }
        }

        for(auto&& [name, joint_info] : joint_infos)
        {
            auto it = link_geo_slots.find(joint_info.child_link_name);
            if(it == link_geo_slots.end())
                continue;
            auto& slot  = it->second;
            auto  trans = slot->geometry().instances().find<Matrix4x4>(attr);
            auto  trans_view = view(*trans);
            trans_view[0]    = URDF2UIPC * joint_info.global_trans;
        }
    }

    void sync_visual_mesh() const
    {
        // if visual mesh is not loaded, do nothing
        if(link_vis_geo_slots.empty())
            return;

        for(auto&& [name, slot] : link_geo_slots)
        {
            auto& vis_slot = link_vis_geo_slots.at(name);
            auto& vis_geo  = vis_slot->geometry();
            auto& geo      = slot->geometry();
            auto  dst_view = view(vis_geo.transforms());
            auto  src_view = view(geo.transforms());
            std::copy(src_view.begin(), src_view.end(), dst_view.begin());
        }
    }
};

UrdfController::UrdfController(S<Impl> impl)
    : m_impl{std::move(impl)}
{
}

UrdfController::~UrdfController() = default;

void UrdfController::move_root(const Vector3& xyz, const Vector3& rpy) const
{
    m_impl->move_root(xyz, rpy);
}

void UrdfController::rotate_to(std::string_view joint_name, Float angle) const
{
    m_impl->rotate_to(joint_name, angle);
}

void UrdfController::apply_to(std::string_view attr) const
{
    m_impl->apply_to(attr);
}

S<geometry::ImplicitGeometrySlot> UrdfController::revolute_joints() const
{
    return m_impl->revolute_joints;
}

vector<S<geometry::SimplicialComplexSlot>> UrdfController::links() const
{
    vector<S<geometry::SimplicialComplexSlot>> links;
    links.reserve(m_impl->link_geo_slots.size());
    for(auto&& [name, slot] : m_impl->link_geo_slots)
    {
        links.push_back(slot);
    }
    return links;
}

void UrdfController::sync_visual_mesh() const
{
    m_impl->sync_visual_mesh();
}

UrdfController::UrdfController(const UrdfController& other)
    : m_impl{uipc::make_shared<Impl>()}
{
    *m_impl = *other.m_impl;
}

class UrdfIO::Impl
{
  public:
    Impl(const Json& config)
    {
        load_visual_mesh = config["load_visual_mesh"].get<bool>();
    }

    bool load_visual_mesh = true;

    S<UrdfController::Impl> ctrl;

    UrdfController read(uipc::core::Object& object, std::string_view urdf_path)
    {
        ctrl = uipc::make_shared<UrdfController::Impl>();

        if(!fs::exists(urdf_path))
        {
            throw UrdfIOError("URDF file does not exist: " + std::string{urdf_path});
        }

        auto& model = ctrl->model;
        model       = urdf::parseURDFFile(std::string{urdf_path});

        if(!model)
        {
            throw UrdfIOError("Failed to parse URDF file: " + std::string{urdf_path});
        }

        ctrl->urdf_folder = fs::path{urdf_path}.parent_path();


        std::map<std::string, urdf::LinkSharedPtr>&  links  = model->links_;
        std::map<std::string, urdf::JointSharedPtr>& joints = model->joints_;

        logger::info(URDFIO_INFO "Model name: {}", model->name_);
        logger::info(URDFIO_INFO "Number of links: {}", links.size());
        logger::info(URDFIO_INFO "Number of joints: {}", joints.size());
        logger::info(URDFIO_INFO "Processing links and joints...");

        // 1) Load link meshes
        auto& link_geo_slots     = ctrl->link_geo_slots;
        auto& link_vis_geo_slots = ctrl->link_vis_geo_slots;
        auto& link_geo_id_map    = ctrl->link_geo_id_map;
        auto& root_name          = model->root_link_->name;
        for(const auto& [name, link] : links)
        {
            // Process each link
            logger::info(URDFIO_INFO "- Link: {}", name);
            if(!check_link(link))
                continue;

            auto sc = load_link_mesh(
                link, *std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry));

            {
                auto is_visible = sc.meta().create<IndexT>("is_visible", 0);
                geometry::view(*is_visible)[0] = load_visual_mesh ? 0 : 1;
            }

            auto&& [geo_slot, rest_geo_slot] = object.geometries().create(sc);
            link_geo_slots[name]             = geo_slot;
            auto id                          = geo_slot->id();
            link_geo_id_map[name]            = id;

            if(load_visual_mesh)
            {
                auto vis_sc = load_link_mesh(
                    link, *std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry));

                auto is_visible = vis_sc.meta().create<IndexT>("is_visible", 0);
                geometry::view(*is_visible)[0] = 1;

                auto&& [vis_geo_slot, rest_vis_geo_slot] =
                    object.geometries().create(vis_sc);
                link_vis_geo_slots[name] = vis_geo_slot;
            }
        }

        // 2) build joints
        build_joint_infos();

        // 3) Create revolute joint geometry
        auto&                      revolute_joints = ctrl->revolute_joints;
        geometry::ImplicitGeometry revolute_joint_geometry =
            create_revolute_geometry(link_geo_id_map);
        auto&& [r_geo_slot, r_rest_geo_slot] =
            object.geometries().create(revolute_joint_geometry);

        revolute_joints = r_geo_slot;

        ctrl->apply_to(builtin::transform);  // apply the transform to the geometry
        return UrdfController{std::move(ctrl)};
    }

    bool check_link(const urdf::LinkSharedPtr& link)
    {
        if(link->collision == nullptr)
        {
            logger::info(URDFIO_INFO "> Link has no collision geometry, skip.");
            return false;
        }

        if(link->collision->geometry == nullptr)
        {
            logger::info(URDFIO_INFO "> Link has no geometry, skip.");
            return false;
        }

        auto link_type = link->collision->geometry->type;
        if(link_type != urdf::Geometry::MESH)
        {
            logger::info(URDFIO_INFO "> Link type {} is not supported yet, skip.",
                         magic_enum::enum_name(link_type));
            return false;
        }

        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);

        if(mesh == nullptr)
        {
            logger::info(URDFIO_INFO "> Link mesh is null, skip.");
            return false;
        }

        if(mesh->filename.empty())
        {
            logger::info(URDFIO_INFO "> Link mesh filename is empty, skip.");
            return false;
        }

        return true;
    }

    geometry::SimplicialComplex load_link_mesh(const urdf::LinkSharedPtr& link,
                                               const urdf::Mesh&          mesh)
    {
        auto& urdf_folder = ctrl->urdf_folder;
        // find first "://"
        auto        pos = mesh.filename.find("://");
        std::string f;
        if(pos != std::string::npos)
        {
            f = fs::canonical(urdf_folder / mesh.filename.substr(pos + 3)).string();
        }
        else
        {
            f = fs::canonical(urdf_folder / mesh.filename).string();
        }

        auto& scale = mesh.scale;

        Transform t = Transform::Identity();
        t           = to_affine(link->collision->origin);
        t.scale(Vector3{scale.x, scale.y, scale.z});

        Matrix4x4 F = t.matrix();

        geometry::SimplicialComplexIO sio{F};
        logger::info(URDFIO_INFO "Loading mesh file: {}", f);
        geometry::SimplicialComplex sc = sio.read(f);

        geometry::label_surface(sc);

        auto urdf_type = sc.meta().create<std::string>("urdf/type", "none");
        view(*urdf_type)[0] = "link";

        auto urdf_name = sc.meta().create<std::string>("urdf/name", "none");
        view(*urdf_name)[0] = link->name;

        auto name      = sc.meta().create<std::string>("name", "none");
        view(*name)[0] = link->name;

        auto urdf_filename = sc.meta().create<std::string>("urdf/filename", "");
        view(*urdf_filename)[0] = f;

        auto& model = ctrl->model;
        if(link->name == model->root_link_->name)
        {
            ctrl->root_link_transform = t.matrix();
        }
        return sc;
    }

    void build_joint_infos()
    {
        auto& model                = ctrl->model;
        auto& revolute_joint_infos = ctrl->revolute_joint_infos;
        auto& fixed_joint_infos    = ctrl->fixed_joint_infos;
        auto& joint_infos          = ctrl->joint_infos;

        for(auto&& [name, joint] : model->joints_)
        {
            if(joint->type == urdf::Joint::REVOLUTE)
            {
                auto& R = revolute_joint_infos[joint->name];
                logger::info("Revolute Joint: {}", joint->name);

                R.local_axis = Vector3{joint->axis.x, joint->axis.y, joint->axis.z};
                R.angle = 0.0;

                if(joint->limits == nullptr)
                {
                    R.limits = Vector2{1, -1};  // revert the min max to tell
                                                // that the joint is not limited
                }
                else
                {
                    R.limits = Vector2{joint->limits->lower, joint->limits->upper};
                }

                if(joint->mimic != nullptr)
                {
                    R.mimic_joint_name = joint->mimic->joint_name;
                    R.mimic_multiplier_offset =
                        Vector2{joint->mimic->multiplier, joint->mimic->offset};
                }
            }
            else if(joint->type == urdf::Joint::FIXED)
            {
                auto& F = fixed_joint_infos[joint->name];
                logger::info("Fixed Joint: {}", joint->name);
            }
            else
            {
                UIPC_ASSERT(false,
                            "Unsupported Joint {} <{}>",
                            joint->name,
                            magic_enum::enum_name(joint->type));
            }

            UIPC_ASSERT(joint_infos.find(joint->name) == joint_infos.end(),
                        "Joint name {} already exists, invalid urdf",
                        joint->name);

            Matrix4x4 t = to_affine(joint->parent_to_joint_origin_transform);
            auto&     base_joint_info = joint_infos[joint->name];

            base_joint_info.local_trans = t;  // local transform of the joint
            base_joint_info.current_local_trans = t;  // current local transform of the joint
            base_joint_info.global_trans = Matrix4x4::Identity();  // fill it later

            base_joint_info.name             = joint->name;
            base_joint_info.parent_link_name = joint->parent_link_name;
            base_joint_info.child_link_name  = joint->child_link_name;
        }
    }


    geometry::ImplicitGeometry create_revolute_geometry(std::map<std::string, IndexT>& link_geo_id_map)
    {
        auto& model                = ctrl->model;
        auto& revolute_joint_infos = ctrl->revolute_joint_infos;
        auto& joint_infos          = ctrl->joint_infos;

        geometry::ImplicitGeometry revolute_joint_geometry;
        revolute_joint_geometry.instances().resize(revolute_joint_infos.size());

        auto trans = revolute_joint_geometry.instances().create<Matrix4x4>(
            builtin::transform, Matrix4x4::Identity());
        auto tran_view = view(*trans);

        auto link_ids = revolute_joint_geometry.instances().create<Vector2i>(
            "link_ids", -Vector2i::Ones());
        auto link_ids_view = view(*link_ids);

        auto local_axis =
            revolute_joint_geometry.instances().create<Vector3>("local_axis",
                                                                Vector3::Zero());
        auto local_axis_view = view(*local_axis);

        auto limits =
            revolute_joint_geometry.instances().create<Vector2>("limits", Vector2::Zero());
        auto limits_view = view(*limits);

        auto name = revolute_joint_geometry.instances().create<std::string>("name", "");
        auto name_view = view(*name);

        SizeT I = 0;
        for(auto&& [name, r_info] : revolute_joint_infos)
        {
            auto& info       = joint_infos[name];
            tran_view[I]     = info.global_trans;
            link_ids_view[I] = Vector2i{link_geo_id_map[info.parent_link_name],
                                        link_geo_id_map[info.child_link_name]};
            local_axis_view[I] = r_info.local_axis;
            limits_view[I]     = r_info.limits;
            name_view[I]       = info.name;
            ++I;
        }

        return revolute_joint_geometry;
    }
};

UrdfIO::UrdfIO(const Json& config)
    : m_impl(uipc::make_unique<Impl>(config))
{
}

UrdfIO::~UrdfIO() {}

UrdfController UrdfIO::read(uipc::core::Object& object, std::string_view urdf_path)
{
    return m_impl->read(object, urdf_path);
}

Json UrdfIO::default_config()
{
    Json config                = Json::object();
    config["load_visual_mesh"] = true;
    return config;
}
}  // namespace uipc::io