#pragma once
#include <uipc/core/feature.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::core
{
class UIPC_CORE_API AffineBodyStateAccessorFeatureOverrider
{
  public:
    AffineBodyStateAccessorFeatureOverrider()          = default;
    virtual ~AffineBodyStateAccessorFeatureOverrider() = default;

    virtual SizeT get_body_count() = 0;
    virtual geometry::SimplicialComplex do_create_geometry(IndexT body_offset, SizeT body_count);
    virtual void do_copy_from(const geometry::SimplicialComplex& state_geo) = 0;
    virtual void do_copy_to(geometry::SimplicialComplex& state_geo)         = 0;
};

class UIPC_CORE_API AffineBodyStateAccessorFeature final : public Feature
{
  public:
    constexpr static std::string_view FeatureName = "core/affine_body_state_accessor";

    explicit AffineBodyStateAccessorFeature(S<AffineBodyStateAccessorFeatureOverrider> overrider);

    /**
     * @brief Get the number of bodies in the affine body system.
     */
    SizeT body_count() const;

    /**
     * @brief Create a simplicial complex geometry to contain affine body state data.
     *
     * @param body_offset The starting body index in affine body system.
     * @param body_count The number of bodies to include, if body_count == ~0ull, all bodies from body_offset to the end will be included.
     */
    geometry::SimplicialComplex create_geometry(IndexT body_offset = 0,
                                                SizeT  body_count  = ~0ull);

    /**
     * @brief Copy affine body state data from the given geometry.
     *
     * @param state_geo The geometry containing affine body state data to copy from.
     */
    void copy_from(const geometry::SimplicialComplex& state_geo) const;

    /**
     * @brief Copy affine body state data to the given geometry.
     *
     * @param state_geo The geometry to copy affine body state data to.
     */
    void copy_to(geometry::SimplicialComplex& state_geo) const;

  private:
    virtual std::string_view                   get_name() const override;
    S<AffineBodyStateAccessorFeatureOverrider> m_impl;
};
}  // namespace uipc::core
