#pragma once
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/json.h>
#include <numbers>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API SoftTransformConstraint final : public Constraint
{
    using Base = Constraint;

  public:
    SoftTransformConstraint(const Json& config = default_config()) noexcept;

    /**
     * @brief Apply the constraint to the simplicial complex instances.
     *
     * @param sc The simplicial complex to apply the constraint to.
     * @param strength_ratio Constraint strength ratio (eta_p, eta_a), where:
     *   strength_ratio[0] = eta_p: center-of-mass translation constraint strength.
     *   strength_ratio[1] = eta_a: rotation/deformation-about-CM constraint strength.
     *   Both values must be non-negative. Typical range: [0, 100].
     */
    void apply_to(geometry::SimplicialComplex& sc, const Vector2& strength_ratio) const;

    static Json default_config();

  protected:
    U64 get_uid() const noexcept override;

  private:
    Json m_config;
};

class UIPC_CONSTITUTION_API RotatingMotor final : public Constraint
{
    using Base = Constraint;

  public:
    RotatingMotor(const Json& config = default_config()) noexcept;

    void apply_to(geometry::SimplicialComplex& sc,
                  Float                        strength_ratio = 100.0,
                  const Vector3& motor_rot_axis = Vector3::UnitX(),
                  Float          motor_rot_vel  = 2 * std::numbers::pi) const;

    static Json default_config();

    static void animate(geometry::SimplicialComplex& sc, Float dt);

  protected:
    U64 get_uid() const noexcept override;

  private:
    Json m_config;
};

class UIPC_CONSTITUTION_API LinearMotor final : public Constraint
{
    using Base = Constraint;

  public:
    LinearMotor(const Json& config = default_config()) noexcept;

    void apply_to(geometry::SimplicialComplex& sc,
                  Float                        strength_ratio = 100.0,
                  Vector3                      motor_axis = -Vector3::UnitZ(),
                  Float                        motor_vel  = 1.0) const;

    static Json default_config();

    static void animate(geometry::SimplicialComplex& sc, Float dt);

  protected:
    U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution