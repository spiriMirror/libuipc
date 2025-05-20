#pragma once
#include <uipc/core/scene.h>
#include <uipc/common/exception.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::core
{
class UIPC_IO_API SceneIO
{
  public:
    SceneIO(Scene& scene);

    /**
     * @brief Write the surface of the scene to a file.
     * Supported formats:
     * - .obj
     */
    void write_surface(std::string_view filename);

    /**
     * @brief Get the surface of the scene in the form of a simplicial complex allow
     * to specify the dimension of the surface.
     * -  0: point cloud
     * -  1: line mesh
     * -  2: triangle mesh
     * - -1: all dimensions
     * \param dim
     * \return 
     */
    geometry::SimplicialComplex simplicial_surface(IndexT dim = -1) const;

    /**
     * @brief Load a scene from a file.
     * 
     * Supported formats:
     * - .json
     * - .bson
     * 
     * @param filename
     * @return 
     */
    static Scene load(std::string_view filename);

    /**
     * @brief Save the scene to a file.
     * 
     * Supported formats:
     * - .json
     * - .bson
     * 
     * @param scene
     * @param filename
     */
    static void save(const Scene& scene, std::string_view filename);

    /**
     * @brief Save the scene to a file.
     * 
     * Supported formats:
     * - .json
     * - .bson
     * 
     * @param filename
     */
    void save(std::string_view filename) const;

    /**
     * @brief Convert the scene to a JSON object.
     * 
     * @return 
     */
    Json to_json() const;

    /**
     * @brief Convert a JSON object to a scene.
     * 
     * @param json
     * @return 
     */
    static Scene from_json(const Json& json);

  private:
    Scene& m_scene;
    void   write_surface_obj(std::string_view filename);
};

class UIPC_IO_API SceneIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::core
