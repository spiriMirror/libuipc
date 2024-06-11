#include <catch.hpp>

std::uint64_t Fibonacci(std::uint64_t number)
{
    return number < 2 ? 1 : Fibonacci(number - 1) + Fibonacci(number - 2);
}

TEST_CASE("Fibonacci")
{
    CHECK(Fibonacci(0) == 1);
    // some more asserts..
    CHECK(Fibonacci(5) == 8);
    // some more asserts..

    // now let's benchmark:
    BENCHMARK("Fibonacci 20")
    {
        return Fibonacci(20);
    };

    BENCHMARK("Fibonacci 25")
    {
        return Fibonacci(25);
    };

    BENCHMARK("Fibonacci 30")
    {
        return Fibonacci(30);
    };

    BENCHMARK("Fibonacci 35")
    {
        return Fibonacci(35);
    };
}