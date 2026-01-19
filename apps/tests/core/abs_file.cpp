#include <app/app.h>
#include <filesystem>

TEST_CASE("abs_file", "[core]")
{
    // Check if __FILE__ is absolute
    std::filesystem::path file_path(__FILE__);

    // The __FILE__ macro should expand to an absolute path
    REQUIRE(file_path.is_absolute());
    
    // Verify it's a valid path
    REQUIRE(std::filesystem::exists(file_path));
    
    // Verify it points to this file
    REQUIRE(file_path.filename() == "abs_file.cpp");
}

