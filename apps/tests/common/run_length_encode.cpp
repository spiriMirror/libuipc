#include <app/test_common.h>
#include <uipc/common/algorithm/run_length_encode.h>

using namespace uipc;

TEST_CASE("run_length_encode", "[algorithm]")
{
    std::vector<int> input = {1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3};
    std::vector<int> output;
    output.reserve(input.size());
    std::vector<int> counts;
    counts.reserve(input.size());

    auto count = run_length_encode(input.begin(),
                                   input.end(),
                                   std::back_inserter(output),
                                   std::back_inserter(counts));

    REQUIRE(count == 3);
    REQUIRE(output == std::vector<int>{1, 2, 3});
    REQUIRE(counts == std::vector<int>{4, 3, 4});


    input = {1, 2, 3, 4, 5};
    output.resize(input.size());
    counts.resize(input.size());
    count = run_length_encode(input.begin(), input.end(), output.begin(), counts.begin());
    output.resize(count);
    counts.resize(count);

    REQUIRE(count == 5);
    REQUIRE(output == std::vector<int>{1, 2, 3, 4, 5});
    REQUIRE(counts == std::vector<int>{1, 1, 1, 1, 1});
}
