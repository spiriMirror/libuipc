#include <cuda_tool/cuda_tool.h>

#include <app/app.h>

namespace
{
using uipc::backend::cuda_tool::DeviceBuffer;

TEST_CASE("cuda_tool DeviceVector growth policies", "[cuda][cuda_tool][buffer]")
{
    SECTION("resize keeps value initialization")
    {
        DeviceBuffer<int> values;
        values.resize(4, 7);
        values.resize(2);
        values.resize(4);

        std::vector<int> host;
        values.copy_to(host);
        REQUIRE(host == std::vector<int>{7, 7, 0, 0});
    }

    SECTION("resize with an explicit value initializes only once logically")
    {
        DeviceBuffer<int> values;
        values.resize(5, 9);

        std::vector<int> host;
        values.copy_to(host);
        REQUIRE(host == std::vector<int>(5, 9));
    }

    SECTION("discard resize grows from the latest requirement")
    {
        DeviceBuffer<int> values;
        values.resize_discard(10);

        REQUIRE(values.size() == 10);
        REQUIRE(values.capacity() == 15);

        auto* pointer = values.data();
        values.resize_discard(3);
        values.resize_discard(12);
        REQUIRE(values.data() == pointer);
        REQUIRE(values.capacity() == 15);

        values.resize_discard(16);
        REQUIRE(values.size() == 16);
        REQUIRE(values.capacity() == 24);
    }

    SECTION("preserve resize retains the previous logical range")
    {
        DeviceBuffer<int> values;
        values.resize(4, 3);
        values.resize_preserve(10);

        REQUIRE(values.size() == 10);
        REQUIRE(values.capacity() == 15);

        std::vector<int> host(4);
        values.cview(0, 4).copy_to(host.data());
        REQUIRE(host == std::vector<int>(4, 3));
    }

    SECTION("amortized reserve does not change the logical size")
    {
        DeviceBuffer<int> values;
        values.resize(4, 5);
        values.reserve_amortized(10);

        REQUIRE(values.size() == 4);
        REQUIRE(values.capacity() == 15);

        std::vector<int> host;
        values.copy_to(host);
        REQUIRE(host == std::vector<int>(4, 5));
    }

    SECTION("discard reserve keeps the logical size and requested capacity")
    {
        DeviceBuffer<int> values;
        values.resize(4, 5);
        values.reserve_discard(11);

        REQUIRE(values.size() == 4);
        REQUIRE(values.capacity() == 11);

        values.resize_discard(11);
        REQUIRE(values.capacity() == 11);
    }
}
}  // namespace
