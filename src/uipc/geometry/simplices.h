#pragma once
#include <uipc/geometry/topo_elements.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>

namespace uipc::geometry
{
/**
 * @brief An abstract class for simplices, special type of topological elements.
 * 
 */
class ISimplices : public ITopoElements
{
  public:
    /**
     * @brief Get the dimension of the simplices.
     *
     * | Dimension | Type of simplices |
     * |-----------|-------------------|
     * |    0      | Vertices          |
     * |    1      | Edges             |
     * |    2      | Triangles         |
     * |    3      | Tetrahedra        |
     *
     * @return the dimension of the simplices
     */
    [[nodiscard]] IndexT dim() const;

  protected:
    S<ISimplices>  clone() const;
    virtual SizeT  get_tuple_size() const override;
    virtual SizeT  get_tuple_size(IndexT i) const override;
    virtual IndexT get_dim() const = 0;
};

/**
 * @brief A collection of vertices.
 *
 * $V=\{0,1,2,...,N-1\}$, where $N$ is the number of vertices.
 * Normally, we don't store the vertice indices, because the indices is just iota $[0, N)$.
 */
class Vertices final : public ISimplices
{
  public:
    Vertices() = default;
    /**
     * @brief Get the const view of the vertices
     */
    [[nodiscard]] span<const IndexT> view() const;

    /**
     * @brief Get the non-const view of the vertices
     */
    friend [[nodiscard]] span<IndexT> view(Vertices& vertices)
    {
        return vertices.view();
    }

  protected:
    virtual IndexT           get_dim() const override;
    virtual SizeT            get_size() const override;
    virtual void             do_resize(SizeT N) override;
    virtual void             do_clear() override;
    virtual S<ITopoElements> do_clone() const override;
    virtual void             do_reserve(SizeT N) override;

  private:
    size_t                     m_size = 0;
    mutable vector<IndexT>     m_simplices;
    [[nodiscard]] span<IndexT> view();
};

/**
 * @brief General class to represent simplices, typically used for edges, triangles, tetrahedra.
 * 
 * @tparam N 
 */
template <IndexT N>
class Simplices final : public ISimplices
{
  public:
    Simplices() = default;
    /**
     * @brief Get the const view of the simplices, this method generates no data clone.
     *
     * @return A span of simplices
     */
    [[nodiscard]] span<const Vector<IndexT, N + 1>> view() const;
    /**
     * @brief Get the non-const view of the simplices, this method may potentially generate data clone.
     *
     * @return A span of simplices
     */
    friend [[nodiscard]] span<Vector<IndexT, N + 1>> view(Simplices& simplices)
    {
        return simplices.view();
    }

  private:
    [[nodiscard]] span<Vector<IndexT, N + 1>> view();

    vector<Vector<IndexT, N + 1>> m_simplices;

  protected:
    virtual IndexT           get_dim() const override;
    virtual SizeT            get_size() const override;
    virtual void             do_resize(SizeT N) override;
    virtual void             do_clear() override;
    virtual S<ITopoElements> do_clone() const override;
    virtual void             do_reserve(SizeT N) override;
};

/**
 * @brief A collection of edges.
 * 
 * $E=\{(i,j) \mid i,j\in V, i\neq j\}$, where $V$ is the set of vertices.
 */
using Edges = Simplices<1>;
/**
 * @brief A collection of triangles. 
 * 
 * $F=\{(i,j,k) \mid i,j,k\in V, i\neq j\neq k\}$, where $V$ is the set of vertices.
 */
using Triangles = Simplices<2>;
/**
 * @brief A collection of tetrahedra.
 * 
 * $T=\{(i,j,k,l) \mid i,j,k,l\in V, i\neq j\neq k\neq l\}$, where $V$ is the set of vertices.
 */
using Tetrahedra = Simplices<3>;
}  // namespace uipc::geometry

#include "details/simplices.inl"