//#include <uipc/core/scene_archieve.h>
//#include <uipc/core/scene_factory.h>


// A Json representation of a Scene may look like this:
//
//  {
//      __meta__:
//      {
//          type:"Scene"
//      },
//      __data__:
//      {
//          config:{ ... },
//
//          contact_tabular:
//          {
//              elements:{}
//          }
//
//          objects:
//          [
//              OBJECT Json
//          ]
//
//          geometry_atlas:
//          {
//              ATLAS Json
//          }
//     }
//  }

// A Json representation of the scene modification may look like this:
//  {
//      __meta__:
//      {
//          type:"SceneArchieve::Commit",
//      },
//      __data__:
//      {
//
//      }
//  }
//
//
//

namespace uipc::core
{
//class SceneArchieve::Impl
//{
//  public:
//    Impl(Scene& scene)
//        : m_scene(scene)
//    {
//        SceneFactory sf;
//    }
//
//    Json push()
//    {
//        SceneFactory sf;
//        auto         j = sf.to_json(m_scene);
//        return j;
//    }
//
//    void pull(const Json& j) {}
//
//
//    Scene& m_scene;
//
//    geometry::GeometryCollection geometry_collection;
//    geometry::GeometryCollection rest_geometry_collection;
//};
//
//SceneArchieve::SceneArchieve(Scene& scene)
//    : m_impl{uipc::make_unique<Impl>(scene)}
//{
//}
//
//SceneArchieve::~SceneArchieve() = default;
//
//const Json& SceneArchieve::init()
//{
//    return m_impl->init();
//}
//
//Json SceneArchieve::push()
//{
//    return m_impl->push();
//}
//
//void SceneArchieve::pull(const Json& json)
//{
//    m_impl->pull(json);
//}
}  // namespace uipc::core