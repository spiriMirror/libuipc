#pragma once
#include <type_define.h>
#include <muda/type_define.h>

namespace uipc::backend::cuda
{
// Currently Unused. May be removed in the future.
class SimplexContactConstraint
{
  public:
    UIPC_GENERIC SimplexContactConstraint() noexcept;

    enum class Type : int
    {
        None = 0,
        EE,
        PE_in_EE,
        PP_in_EE,
        PT,
        PE_in_PT,
        PP_in_PT,
        PE,
        PP_in_PE,
        PP
    };

    // Creation:

    UIPC_GENERIC static SimplexContactConstraint EE(const Vector2i& e0,
                                                    const Vector2i& e1) noexcept;
    UIPC_GENERIC static SimplexContactConstraint PE_in_EE(IndexT          p,
                                                          const Vector2i& e,
                                                          IndexT inactive_p) noexcept;
    UIPC_GENERIC static SimplexContactConstraint PP_in_EE(IndexT p_e0,
                                                          IndexT p_e1,
                                                          IndexT inactive_p_e0,
                                                          IndexT inactive_p_e1) noexcept;

    UIPC_GENERIC static SimplexContactConstraint PT(IndexT p, const Vector3i& T) noexcept;
    UIPC_GENERIC static SimplexContactConstraint PE_in_PT(IndexT          p,
                                                          const Vector2i& e,
                                                          IndexT inactive_p) noexcept;
    UIPC_GENERIC static SimplexContactConstraint PP_in_PT(IndexT p,
                                                          IndexT p_t,
                                                          const Vector2i& inactive_e) noexcept;

    UIPC_GENERIC static SimplexContactConstraint PE(IndexT p, const Vector2i& e) noexcept;
    UIPC_GENERIC static SimplexContactConstraint PP_in_PE(IndexT p, IndexT p_e, IndexT inactive_p) noexcept;

    UIPC_GENERIC static SimplexContactConstraint PP(IndexT p0, IndexT p1) noexcept;

    UIPC_GENERIC static SimplexContactConstraint None() noexcept;
    UIPC_GENERIC                                 operator bool() const noexcept;

    // Accessors:

    UIPC_GENERIC Type type() const noexcept;
    UIPC_GENERIC bool is_none() const noexcept;

    UIPC_GENERIC void as_EE(Vector2i& e0, Vector2i& e1) const noexcept;
    UIPC_GENERIC void as_PE_in_EE(IndexT& p, Vector2i& e, IndexT& inactive_p) const noexcept;
    UIPC_GENERIC void as_PP_in_EE(IndexT& p_e0,
                                  IndexT& p_e1,
                                  IndexT& inactive_p_e0,
                                  IndexT& inactive_p_e1) const noexcept;

    UIPC_GENERIC void as_PT(IndexT& p, Vector3i& T) const noexcept;
    UIPC_GENERIC void as_PE_in_PT(IndexT& p, Vector2i& e, IndexT& inactive_p) const noexcept;
    UIPC_GENERIC void as_PP_in_PT(IndexT& p, IndexT& p_t, Vector2i& inactive_e) const noexcept;

    UIPC_GENERIC void as_PE(IndexT& p, Vector2i& e) const noexcept;
    UIPC_GENERIC void as_PP_in_PE(IndexT& p, IndexT& p_e, IndexT& inactive_p) const noexcept;

    UIPC_GENERIC void as_PP(IndexT& p0, IndexT& p1) const noexcept;

    UIPC_GENERIC Vector4i EE_indices() const noexcept;
    UIPC_GENERIC Vector4i PT_indices() const noexcept;
    UIPC_GENERIC Vector3i PE_indices() const noexcept;
    UIPC_GENERIC Vector2i PP_indices() const noexcept;

  private:
    Vector4i     m_data;
    UIPC_GENERIC SimplexContactConstraint(const Vector4i& data) noexcept;
};
}  // namespace uipc::backend::cuda
