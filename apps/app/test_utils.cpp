#include <app/test_utils.h>
#include <uipc/core/scene.h>
#include <fstream>
#include <fmt/format.h>

namespace uipc::test
{
Json Scene::default_config()
{
    Json config = uipc::core::Scene::default_config();
    config["extras"]["strict_mode"]["enable"] = true;
    config["line_search"]["max_iter"]         = 8;
    config["newton"]["max_iter"]              = 16;
    return config;
}

void Scene::dump_config(const Json& config, std::string_view workspace)
{
    auto          filename = fmt::format("{}config.json", workspace);
    std::ofstream ofs(filename);
    ofs << config.dump(4);
    logger::info("Scene Config dumped to {}", filename);
}
}  // namespace uipc::test
