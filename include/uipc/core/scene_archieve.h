//#pragma once
//#include <uipc/core/scene.h>
//
//namespace uipc::core
//{
//class UIPC_CORE_API SceneArchieve
//{
//    class Impl;
//
//  public:
//    SceneArchieve(Scene& scene);
//    ~SceneArchieve();
//
//    /**
//     * @brief Create a Json of the whole scene 
//     */
//    const Json& init();
//
//    /**
//     * @brief Create a Json of the modification of the scene
//     */
//    Json push();
//
//    /**
//     * @brief Update the scene with the given Json file
//     */
//    void pull(const Json& json);
//
//  private:
//    U<Impl> m_impl;
//};
//}  // namespace uipc::core
