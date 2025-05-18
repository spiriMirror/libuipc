#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/vector.h>

namespace uipc::core::internal
{
class Scene;
}
namespace uipc::core
{
class SceneFactory;

class SceneSnapshot;

class ObjectSnapshot
{
    friend class SceneSnapshot;
    friend class SceneFactory;
    friend class ObjectCollection;
    friend class internal::Scene;
    friend class Object;

  private:
    IndexT         m_id;
    std::string    m_name;
    vector<IndexT> m_geometries;
};
}  // namespace uipc::core
