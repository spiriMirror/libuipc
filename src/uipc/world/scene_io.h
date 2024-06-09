#pragma once
#include <uipc/world/scene.h>
#include <uipc/common/exception.h>

namespace uipc::world
{
class SceneIO
{
  public:
    SceneIO(Scene& scene);

    void write_surface(std::string_view filename);
    void write_surface_as_obj(std::string_view filename);

  private:
    Scene& m_scene;
};

class SceneIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::world
