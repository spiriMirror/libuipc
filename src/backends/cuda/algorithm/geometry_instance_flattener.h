#pragma once
#include <tuple>
#include <utility>
#include <uipc/geometry/geometry.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>

namespace uipc::backend::cuda
{
template <std::derived_from<geometry::Geometry> GeometryT>
class GeometryInstanceFlattener
{
  public:
    GeometryInstanceFlattener(span<GeometryT*> geos)
        : m_geos(geos)
    {
    }

    size_t compute_instance_count() const
    {
        size_t count = 0;
        for(auto geo : m_geos)
        {
            count += geo->instances().size();
        }
        return count;
    }

    vector<size_t> compute_instance_to_gemetry() const
    {
        vector<size_t> map;
        map.reserve(compute_instance_count());

        for(auto&& [I, geo] : enumerate(m_geos))
        {
            auto N = geo->instances().size();
            for(auto J : range(N))
            {
                map.push_back(I);
            }
        }
        return map;
    }

    vector<size_t> compute_geometry_to_instance_offset() const
    {
        vector<size_t> offsets;
        offsets.reserve(m_geos.size());

        size_t offset = 0;
        for(auto geo : m_geos)
        {
            offsets.push_back(offset);
            offset += geo->instances().size();
        }

        return offsets;
    }

    vector<size_t> compute_geometry_to_instance_count() const
    {
        vector<size_t> counts;
        counts.reserve(m_geos.size());

        for(auto geo : m_geos)
        {
            counts.push_back(geo->instances().size());
        }

        return counts;
    }

    /**
     * @brief Flatten the geometry instances and apply the looper to each instance.
     * 
     * @code
     *  flattener.flatten(
     *      [&](geometry::Geometry* geo)
     *      {
     *          auto transform = geo->instances().find<Matrix4x4>(builtin::transform);
     *          return std::make_tuple(transform->view());
     *      },
     *      [&](const Matrix4x4& transform)
     *      {
     *          std::cout << transform << std::endl;
     *      });
     * @endcode
     * 
     * @param AttributesGetter A callable object that takes a geometry and returns a tuple of span<T>s, size of the span is the number of instances in the geometry.
     * @param Looper A callable object that takes the attributes of each instance, arguments are the elements of span<T>s returned by AttributesGetter.
     */
    template <typename AttributesGetter, typename Looper>
    void flatten(AttributesGetter&& Getter, Looper&& looper)
    {
        for(auto&& [I, geo] : enumerate(m_geos))
        {
            auto N = geo->instances().size();

            // return a tuple of span<T>s.
            std::tuple spans = Getter(geo);

            for(auto&& J : range(N))
            {
                auto loop = [&](auto&&... spans) { looper(spans[J]...); };
                std::apply(loop, spans);
            }
        }
    }

  private:
    span<GeometryT*> m_geos;
};

template <std::derived_from<geometry::Geometry> GeometryT>
GeometryInstanceFlattener(span<GeometryT*> geos)
    -> GeometryInstanceFlattener<GeometryT>;

}  // namespace uipc::backend::cuda
