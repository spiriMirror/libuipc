#include <app/app.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <fstream>


TEST_CASE("scene", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene scene;

    auto object = scene.objects().create("cube");

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    auto [geo, rest_geo] = object->geometries().create(mesh);

    const auto& const_obj = *object;

    auto ID = object->id();
    scene.objects().destroy(ID);

    REQUIRE_ONCE_WARN(scene.objects().destroy(ID));

    object = scene.objects().create("cube1");

    // create two geometries
    object->geometries().create(mesh);
    object->geometries().create(mesh);

    REQUIRE(object->geometries().ids().size() == 2);
    REQUIRE(object->geometries().ids()[0] == 1);
    REQUIRE(object->geometries().ids()[1] == 2);
}


TEST_CASE("pending_sequence", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene scene;
    auto  object = scene.objects().create("cube");
    REQUIRE(object->id() == 0);

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    auto [geo, rest_geo] = object->geometries().create(mesh);

    backend::SceneVisitor visitor{scene};
    visitor.begin_pending();

    auto new_object = scene.objects().create("cube2");
    new_object->geometries().create(mesh);
    REQUIRE(new_object->id() == 1);


    // only geometries will be added to pending list

    // object is not added to pending list
    // so the creation of object is immediate
    REQUIRE(scene.objects().size() == 2);

    // the geometries are added to pending list
    // so the creation of geometries is delayed
    REQUIRE(visitor.geometries().size() == 1);
    REQUIRE(visitor.pending_geometries().size() == 1);
    REQUIRE(visitor.rest_geometries().size() == 1);
    REQUIRE(visitor.pending_rest_geometries().size() == 1);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    // compute_contact the pending list
    visitor.solve_pending();

    // all the pending geometries are created
    REQUIRE(visitor.geometries().size() == 2);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.rest_geometries().size() == 2);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    visitor.begin_pending();

    // destroy the object
    scene.objects().destroy(new_object->id());

    // only geometries will be added to pending list

    // object is not added to pending list
    // so the destruction of object is immediate
    REQUIRE(scene.objects().size() == 1);

    // the geometries are added to pending list
    // so the destruction of geometries is delayed
    REQUIRE(visitor.geometries().size() == 2);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.rest_geometries().size() == 2);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 1);

    // compute_contact the pending list
    visitor.solve_pending();

    // all the pending geometries are destroyed
    REQUIRE(visitor.geometries().size() == 1);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.rest_geometries().size() == 1);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    visitor.begin_pending();

    // create a new object immediately
    new_object = scene.objects().create("cube3");

    REQUIRE(new_object->id() == 2);
    REQUIRE(scene.objects().size() == 2);

    // create 3 geometries in pending list
    new_object->geometries().create(mesh);
    new_object->geometries().create(mesh);
    new_object->geometries().create(mesh);

    REQUIRE(visitor.pending_geometries().size() == 3);
    REQUIRE(visitor.pending_rest_geometries().size() == 3);

    // destroy the object
    REQUIRE_ALL_INFO(scene.objects().destroy(new_object->id()));

    // so the geometries are first added to pending list
    // and then removed from pending list
    // so we can't see the geometries in pending create/destroy list
    REQUIRE(visitor.pending_destroy_ids().size() == 0);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
}

TEST_CASE("pending_create", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene                 scene;
    backend::SceneVisitor visitor{scene};

    auto object = scene.objects().create("cube");
    REQUIRE(object->id() == 0);

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto [geo, rest_geo] = object->geometries().create(mesh);

    // because now we don't start pending, so the geometries are created immediately
    {
        REQUIRE(visitor.geometries().size() == 1);
        REQUIRE(visitor.rest_geometries().size() == 1);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
    }

    visitor.begin_pending();

    // now we start pending, so the geometries are added to pending list
    {
        auto new_object = scene.objects().create("cube2");
        REQUIRE(scene.objects().size() == 2);

        new_object->geometries().create(mesh);

        new_object = scene.objects().create("cube3");
        REQUIRE(scene.objects().size() == 3);

        new_object->geometries().create(mesh);
        new_object->geometries().create(mesh);
        new_object->geometries().create(mesh);

        REQUIRE(visitor.pending_geometries().size() == 4);
        REQUIRE(visitor.pending_rest_geometries().size() == 4);
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
    }

    visitor.solve_pending();

    {
        REQUIRE(visitor.geometries().size() == 5);
        REQUIRE(visitor.rest_geometries().size() == 5);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
    }
}

TEST_CASE("pending_destroy", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene                 scene;
    backend::SceneVisitor visitor{scene};

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));


    constexpr int N = 5;

    vector<IndexT> ids;
    vector<IndexT> geo_ids;

    for(int i = 0; i < N; i++)
    {
        auto new_object = scene.objects().create("cube2");
        new_object->geometries().create(mesh);
        ids.push_back(new_object->id());
        for(auto id : new_object->geometries().ids())
        {
            geo_ids.push_back(id);
        }
    }
    REQUIRE(scene.objects().size() == N);
    REQUIRE(visitor.geometries().size() == N);
    REQUIRE(visitor.rest_geometries().size() == N);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    visitor.begin_pending();

    {
        for(auto id : ids)
        {
            scene.objects().destroy(id);
        }

        // immediately
        REQUIRE(scene.objects().size() == 0);

        // pending
        REQUIRE(visitor.geometries().size() == N);
        REQUIRE(visitor.rest_geometries().size() == N);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
        REQUIRE(visitor.pending_destroy_ids().size() == N);

        for(auto id : geo_ids)
        {
            auto&& [geo, rest_geo] = scene.geometries().find(id);
        }
    }

    visitor.solve_pending();

    {
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
        REQUIRE(visitor.geometries().size() == 0);
        REQUIRE(visitor.rest_geometries().size() == 0);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
    }
}

TEST_CASE("scene_commit", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Scene               scene;
    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto object                    = scene.objects().create("cube");
    auto [geo_slot, rest_geo_slot] = object->geometries().create(mesh);

    SceneSnapshot snapshot{scene};

    SceneFactory sf;
    Scene        new_scene = sf.from_snapshot(snapshot);

    auto& geo      = geo_slot->geometry();
    auto  pos_view = view(geo.positions());
    std::ranges::transform(pos_view.begin(),
                           pos_view.end(),
                           pos_view.begin(),
                           [](const auto& p) { return p + Vector3{1, 1, 1}; });

    SceneSnapshotCommit commit = scene - snapshot;
    REQUIRE(commit.contact_models().attribute_collection().find("topo") == nullptr);
    new_scene.update_from(commit);

    auto new_obj = new_scene.objects().find(object->id());
    REQUIRE(new_obj != nullptr);
    REQUIRE(new_obj->name() == object->name());

    auto [new_geo_slot, new_rest_geo_slot] =
        new_scene.geometries().find(geo_slot->id());

    auto new_geo      = new_geo_slot->geometry().as<SimplicialComplex>();
    auto new_pos_view = new_geo->positions().view();

    REQUIRE(std::ranges::equal(pos_view, new_pos_view));

    auto commit_json = sf.commit_to_json(commit);
    {
        auto output_file = fmt::format("{}/scene_commit.json", output_path);
        std::ofstream ofs(output_file);
        ofs << commit_json.dump(4);
    }

    auto commit2 = sf.commit_from_json(commit_json);
    REQUIRE(commit2.is_valid());

    REQUIRE(commit2.geometries().size() == commit.geometries().size());
    REQUIRE(commit2.rest_geometries().size() == commit.rest_geometries().size());
}

TEST_CASE("scene_commit_preserves_complete_scene_state", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    auto make_point = [](const Vector3& position)
    {
        vector<Vector3> positions{position};
        return pointcloud(positions);
    };

    Scene source;

    auto kept_object = source.objects().create("kept");
    auto kept_slots =
        kept_object->geometries().create(make_point(Vector3{0.0, 0.0, 0.0}),
                                         make_point(Vector3{10.0, 10.0, 10.0}));
    REQUIRE(kept_object->id() == 0);
    REQUIRE(kept_slots.geometry->id() == 0);

    auto removed_object = source.objects().create("removed");
    auto removed_slots =
        removed_object->geometries().create(make_point(Vector3{20.0, 20.0, 20.0}));
    REQUIRE(removed_object->id() == 1);
    REQUIRE(removed_slots.geometry->id() == 1);

    // Leave a trailing allocation gap in the full snapshot.
    auto snapshot_transient = source.objects().create("snapshot_transient");
    auto snapshot_transient_slots =
        snapshot_transient->geometries().create(make_point(Vector3{30.0, 30.0, 30.0}));
    REQUIRE(snapshot_transient->id() == 2);
    REQUIRE(snapshot_transient_slots.geometry->id() == 2);
    source.objects().destroy(snapshot_transient->id());

    SceneSnapshot baseline{source};
    SceneFactory  factory;

    // Full snapshot JSON must preserve the next geometry id, including gaps.
    auto  baseline_json    = factory.to_json(baseline);
    auto  decoded_baseline = factory.from_json(baseline_json);
    Scene baseline_probe   = factory.from_snapshot(decoded_baseline);
    auto  probe_object     = baseline_probe.objects().create("probe");
    auto  probe_slots =
        probe_object->geometries().create(make_point(Vector3{40.0, 40.0, 40.0}));
    REQUIRE(probe_object->id() == 3);
    REQUIRE(probe_slots.geometry->id() == 3);

    Scene receiver = factory.from_snapshot(decoded_baseline);

    view(kept_slots.geometry->geometry().as<SimplicialComplex>()->positions())[0] =
        Vector3{1.0, 2.0, 3.0};
    view(kept_slots.rest_geometry->geometry().as<SimplicialComplex>()->positions())[0] =
        Vector3{4.0, 5.0, 6.0};

    source.objects().destroy(removed_object->id());

    auto added_object = source.objects().create("added");
    auto added_slots =
        added_object->geometries().create(make_point(Vector3{50.0, 50.0, 50.0}));
    REQUIRE(added_object->id() == 3);
    REQUIRE(added_slots.geometry->id() == 3);

    // Leave another trailing allocation gap in the commit.
    auto commit_transient = source.objects().create("commit_transient");
    auto commit_transient_slots =
        commit_transient->geometries().create(make_point(Vector3{60.0, 60.0, 60.0}));
    REQUIRE(commit_transient->id() == 4);
    REQUIRE(commit_transient_slots.geometry->id() == 4);
    source.objects().destroy(commit_transient->id());

    auto& contact_tabular = source.contact_tabular();
    auto  contact_element = contact_tabular.create("test_contact");
    contact_tabular.default_model(0.25, 1234.0, false);
    contact_tabular.insert(contact_tabular.default_element(), contact_element, 0.75, 5678.0, true);

    auto& subscene_tabular = source.subscene_tabular();
    auto  subscene_element = subscene_tabular.create("test_subscene");
    subscene_tabular.insert(subscene_tabular.default_element(), subscene_element, false);

    SceneSnapshotCommit commit = source - baseline;
    REQUIRE(commit.is_valid());
    REQUIRE(commit.geometry_next_id() == 5);
    REQUIRE(commit.rest_geometry_next_id() == 5);
    REQUIRE(commit.removed_geometry_ids().size() == 1);
    REQUIRE(commit.removed_geometry_ids()[0] == 1);
    for(auto&& [name, attribute_commit] : commit.geometries().at(0)->attribute_collections())
    {
        CAPTURE(name,
                attribute_commit->target_size(),
                attribute_commit->attribute_collection().size());
        if(attribute_commit->attribute_collection().attribute_count() > 0)
        {
            REQUIRE(attribute_commit->target_size()
                    == attribute_commit->attribute_collection().size());
        }
    }

    auto decoded_commit = factory.commit_from_json(factory.commit_to_json(commit));
    REQUIRE(decoded_commit.is_valid());
    REQUIRE(decoded_commit.removed_geometry_ids().size() == 1);
    REQUIRE(decoded_commit.removed_geometry_ids()[0] == 1);
    for(auto&& [name, attribute_commit] :
        decoded_commit.geometries().at(0)->attribute_collections())
    {
        CAPTURE(name,
                attribute_commit->target_size(),
                attribute_commit->attribute_collection().size());
        if(attribute_commit->attribute_collection().attribute_count() > 0)
        {
            REQUIRE(attribute_commit->target_size()
                    == attribute_commit->attribute_collection().size());
        }
    }

    receiver.update_from(decoded_commit);

    REQUIRE(receiver.objects().size() == 2);
    REQUIRE(receiver.objects().created_count() == 5);
    REQUIRE(receiver.objects().find(0));
    REQUIRE_FALSE(receiver.objects().find(1));
    REQUIRE_FALSE(receiver.objects().find(2));
    REQUIRE(receiver.objects().find(3));
    REQUIRE_FALSE(receiver.objects().find(4));

    auto receiver_kept = receiver.geometries().find(0);
    REQUIRE(receiver_kept.geometry);
    REQUIRE(receiver_kept.rest_geometry);
    auto receiver_position =
        receiver_kept.geometry->geometry().as<SimplicialComplex>()->positions().view()[0];
    auto receiver_rest_position = receiver_kept.rest_geometry->geometry()
                                      .as<SimplicialComplex>()
                                      ->positions()
                                      .view()[0];
    REQUIRE((receiver_position - Vector3{1.0, 2.0, 3.0}).norm() == Catch::Approx(0.0));
    REQUIRE((receiver_rest_position - Vector3{4.0, 5.0, 6.0}).norm() == Catch::Approx(0.0));

    REQUIRE_FALSE(receiver.geometries().find(1).geometry);
    REQUIRE_FALSE(receiver.geometries().find(2).geometry);
    REQUIRE(receiver.geometries().find(3).geometry);
    REQUIRE_FALSE(receiver.geometries().find(4).geometry);

    auto receiver_default_contact = receiver.contact_tabular().default_model();
    REQUIRE(receiver.contact_tabular().element_count() == 2);
    REQUIRE(receiver.contact_tabular().default_model_is_user_set());
    REQUIRE(receiver_default_contact.friction_rate() == Catch::Approx(0.25));
    REQUIRE(receiver_default_contact.resistance() == Catch::Approx(1234.0));
    REQUIRE_FALSE(receiver_default_contact.is_enabled());
    auto receiver_contact = receiver.contact_tabular().at(0, contact_element.id());
    REQUIRE(receiver_contact.friction_rate() == Catch::Approx(0.75));
    REQUIRE(receiver_contact.resistance() == Catch::Approx(5678.0));
    REQUIRE(receiver_contact.is_enabled());

    REQUIRE(receiver.subscene_tabular().element_count() == 2);
    REQUIRE_FALSE(receiver.subscene_tabular().at(0, subscene_element.id()).is_enabled());

    auto next_object = receiver.objects().create("next");
    auto next_slots =
        next_object->geometries().create(make_point(Vector3{70.0, 70.0, 70.0}));
    REQUIRE(next_object->id() == 5);
    REQUIRE(next_slots.geometry->id() == 5);
    REQUIRE(next_slots.rest_geometry->id() == 5);
}

TEST_CASE("scene_commit_empty", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Scene               scene;
    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto object                    = scene.objects().create("cube");
    auto [geo_slot, rest_geo_slot] = object->geometries().create(mesh);

    SceneSnapshot snapshot0{scene};
    SceneSnapshot snapshot1{scene};

    geo_slot->geometry().vertices().create<Vector3>("myattribute");

    SceneSnapshotCommit ssc = snapshot1 - snapshot0;
    REQUIRE(ssc.contact_models().attribute_collection().attribute_count() == 0);

    SceneSnapshotCommit ssc2 = scene - snapshot1;
    REQUIRE(ssc2.geometries()
                .find(0)
                ->second->attribute_collections()
                .find("vertices")
                ->second->attribute_collection()
                .find("myattribute")
            != nullptr);
}


TEST_CASE("dynamic_dt_config", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene scene;

    // Default dt should be 0.01
    {
        auto dt_slot = scene.config().find<Float>("dt");
        REQUIRE(dt_slot != nullptr);
        REQUIRE(dt_slot->view()[0] == Catch::Approx(0.01));
    }

    // Change dt via config
    {
        auto dt_slot      = scene.config().find<Float>("dt");
        view(*dt_slot)[0] = 0.005;
    }

    // Read back via config
    {
        auto dt_slot = scene.config().find<Float>("dt");
        REQUIRE(dt_slot->view()[0] == Catch::Approx(0.005));
    }

    // Read back via SceneVisitor
    {
        backend::SceneVisitor visitor{scene};
        REQUIRE(visitor.dt() == Catch::Approx(0.005));
    }

    // Change again and verify
    {
        auto dt_slot      = scene.config().find<Float>("dt");
        view(*dt_slot)[0] = 0.02;
    }

    {
        backend::SceneVisitor visitor{scene};
        REQUIRE(visitor.dt() == Catch::Approx(0.02));
    }
}
