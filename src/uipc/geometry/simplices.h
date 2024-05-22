#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <vector>
#include <span>

namespace uipc::geometry
{
/**
 * \brief Interface for simplices
 */
class ISimplices
{
  public:
    virtual IndexT dim() const = 0;
    SizeT          size() const;
    void           resize(SizeT N);
    void           clear();
    S<ISimplices>  clone() const;

    void reserve(SizeT N);

    template <typename Derived>
        requires std::is_base_of_v<ISimplices, Derived>
    Derived& cast();

    template <typename Derived>
        requires std::is_base_of_v<ISimplices, Derived>
    const Derived& cast() const;

  protected:
    virtual SizeT         get_size() const    = 0;
    virtual void          do_resize(SizeT N)  = 0;
    virtual void          do_clear()          = 0;
    virtual S<ISimplices> do_clone() const    = 0;
    virtual void          do_reserve(SizeT N) = 0;
};

class Vertices final : public ISimplices
{
  public:
    Vertices() = default;
    virtual IndexT dim() const override;

    std::span<const IndexT> view() const;
    std::span<IndexT>       view();

  protected:
    virtual SizeT         get_size() const override;
    virtual void          do_resize(SizeT N) override;
    virtual void          do_clear() override;
    virtual S<ISimplices> do_clone() const override;
    virtual void          do_reserve(SizeT N) override;

  private:
    size_t                      m_size = 0;
    mutable std::vector<IndexT> m_simplices;
};

template <IndexT N>
class Simplices final : public ISimplices
{
  public:
    Simplices() = default;
    virtual IndexT dim() const override;

    std::span<const Vector<IndexT, N + 1>> view() const;
    std::span<Vector<IndexT, N + 1>>       view();

  private:
    std::vector<Vector<IndexT, N + 1>> m_simplices;

  protected:
    virtual SizeT         get_size() const override;
    virtual void          do_resize(SizeT N) override;
    virtual void          do_clear() override;
    virtual S<ISimplices> do_clone() const override;
    virtual void          do_reserve(SizeT N) override;
};

using Edges      = Simplices<1>;
using Triangles  = Simplices<2>;
using Tetrahedra = Simplices<3>;
}  // namespace uipc::geometry

#include "details/simplices.inl"