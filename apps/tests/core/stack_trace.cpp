#include <app/test_common.h>
#include <uipc/uipc.h>
#include <numeric>
#include <cpptrace/cpptrace.hpp>
#include <uipc/common/abort.h>

using namespace uipc;
using namespace uipc::geometry;

void stackA()
{
    // test abort with stack trace
    UIPC_ASSERT(false, "Abort here!");
}

void stackB()
{
    stackA();
    fmt::println("this should not be printed");
}

TEST_CASE(".stack_trace", "[.stack_trace]")
{
    stackB();
}
