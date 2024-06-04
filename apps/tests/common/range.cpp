#include <app/test_common.h>
#include <uipc/common/range.h>
#include <algorithm>
#include <numeric>

using namespace uipc;

TEST_CASE("range", "[range]")
{
    constexpr int N = 10;

    auto r = range(N);

    std::vector<int> v;
    v.resize(r.size());

    std::copy(r.begin(), r.end(), v.begin());

    std::vector<int> expected(N);

    std::iota(expected.begin(), expected.end(), 0);

    REQUIRE(v == expected);
}
