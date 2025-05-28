#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>


TEST_CASE("attribute_collection", "[geometry]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;


    SECTION("copy")
    {
        // Behavior of copy assignment operator
        // https://github.com/spiriMirror/libuipc/issues/133

        AttributeCollection foo;
        auto                pos = foo.create<Vector3>("position");

        AttributeCollection bar;
        auto                pos1     = bar.create<Vector3>("position");
        auto                velocity = bar.create<Vector3>("velocity");

        // 1. copy should not change the attribute slot pointer
        foo            = bar;
        auto pos_found = foo.find("position");
        REQUIRE(pos_found.get() == pos.get());

        // 2. create new attribute slot if the destination attribute collection
        // does not have the attribute name
        auto vel_found = foo.find("velocity");
        // a new attribute slot is created
        REQUIRE(vel_found != nullptr);
        // not the same attribute slot pointer
        REQUIRE(vel_found != velocity);

        // 3. remove the attribute slot if the source attribute collection
        // does not have the same attribute name
        foo       = AttributeCollection{};
        auto pos2 = foo.find<Vector3>("position");
        auto vel2 = foo.find<Vector3>("velocity");
        // removed
        REQUIRE(pos2 == nullptr);
        REQUIRE(vel2 == nullptr);
    }

    SECTION("update_from")
    {
        AttributeCollection foo;
        foo.resize(10);
        auto pos = foo.create<Vector3>("position");

        AttributeCollection foo_copy = foo;

        foo.create<Vector3>("velocity");

        AttributeCollectionCommit c = foo - foo_copy;
        foo_copy.update_from(c);

        auto commit_pos = c.attribute_collection().find("position");
        REQUIRE(commit_pos == nullptr);

        auto pos_found = foo_copy.find("position");
        auto vel_found = foo_copy.find("velocity");

        REQUIRE(pos_found != nullptr);
        REQUIRE(vel_found != nullptr);

        foo.destroy("position");

        AttributeCollectionCommit c2 = foo - foo_copy;
        foo_copy.update_from(c2);

        auto pos_found2 = foo_copy.find("position");
        auto vel_found2 = foo_copy.find("velocity");
        // position is removed
        REQUIRE(pos_found2 == nullptr);
        // velocity is not removed
        REQUIRE(vel_found2 != nullptr);
    }

    SECTION("modification_time")
    {
        // Test the modification time of the attribute slot
        AttributeCollection foo;
        foo.resize(10);
        auto pos = foo.create<Vector3>("position");
        auto vel = foo.create<Vector3>("velocity");
        // vel has a later create/modification time than pos
        REQUIRE(vel->last_modified() > pos->last_modified());
        auto new_pos = view(*pos);
        std::ranges::fill(new_pos, Vector3::Ones());
        // pos has a later modification time than vel
        REQUIRE(pos->last_modified() > vel->last_modified());
    }
}
