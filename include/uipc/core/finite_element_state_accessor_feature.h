#pragma once
#include <uipc/core/feature.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::core
{
class UIPC_CORE_API FiniteElementStateAccessorFeatureOverrider
{
  public:
    FiniteElementStateAccessorFeatureOverrider()          = default;
    virtual ~FiniteElementStateAccessorFeatureOverrider() = default;

    virtual SizeT                       get_vertex_count() = 0;
    virtual geometry::SimplicialComplex do_create_geometry(IndexT vertex_offset,
                                                           SizeT  vertex_count);
    virtual void do_copy_from(const geometry::SimplicialComplex& state_geo) = 0;
    virtual void do_copy_to(geometry::SimplicialComplex& state_geo)   = 0;
};

class UIPC_CORE_API FiniteElementStateAccessorFeature final : public Feature
{
  public:
    constexpr static std::string_view FeatureName = "core/finite_element_state_accessor";

    explicit FiniteElementStateAccessorFeature(S<FiniteElementStateAccessorFeatureOverrider> overrider);

    /**
     * @brief Get the number of vertices in the finite element system.
     */
    SizeT vertex_count() const;

    /**
     * @brief Create a simplicial complex geometry to contain finite element state data.
     * 
     * @param vertex_offset The starting vertex index in finite element system.
     * @param vertex_count The number of vertices to include, if vertex_count == ~0ull, all vertices from vertex_offset to the end will be included.
     */
    geometry::SimplicialComplex create_geometry(IndexT vertex_offset = 0,
                                                SizeT  vertex_count  = ~0ull) const;

    /**
     * @brief Copy finite element state data from the given geometry.
     * 
     * @param state_geo The geometry containing finite element state data to copy from.
     */
    void copy_from(const geometry::SimplicialComplex& state_geo) const;

    /**
     * @brief Copy finite element state data to the given geometry.
     *
     * @param state_geo The geometry to copy finite element state data to.
     */
    void copy_to(geometry::SimplicialComplex& state_geo) const;

  private:
    virtual std::string_view                      get_name() const override;
    S<FiniteElementStateAccessorFeatureOverrider> m_impl;
};
}  // namespace uipc::core
