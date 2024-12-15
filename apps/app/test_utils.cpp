#include <app/test_utils.h>
#include <uipc/core/scene.h>

namespace uipc::test
{
Json Scene::default_config()
{
    Json config = uipc::core::Scene::default_config();
    config["extras"]["restrict_mode"]["enable"] = true;
    return config;
}
}  // namespace uipc::test
