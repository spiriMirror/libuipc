#include <app/app.h>
#include <filesystem>

TEST_CASE("relative_source_file", "[core]")
{
    std::filesystem::path rel_path(UIPC_RELATIVE_SOURCE_FILE);

    // The custom macro should expand to a project-relative path
    REQUIRE(rel_path.is_relative());

    // Verify it's a valid path
    auto abs_path = std::filesystem::path(UIPC_PROJECT_DIR) / rel_path;
    REQUIRE(std::filesystem::exists(abs_path));

    // Verify it points to this file
    REQUIRE(rel_path.filename() == "uipc_relative_source_file.cpp");
}


