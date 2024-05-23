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
     * E.g. 0 for vertices, 1 for edges, 2 for triangles, 3 for tetrahedra.
     * 
     * @return IndexT the dimension of the simplices
     */
    [[nodiscard]] IndexT dim() const;

  protected:
    S<ISimplices>  clone() const;
    virtual SizeT  get_tuple_size() const override;
    virtual SizeT  get_tuple_size(IndexT i) const override;
    virtual IndexT get_dim() const = 0;
};

/**
 * @brief A collection of vertices, $V=\{0,1,2,...,N-1\}$
 *
 * Normally, we don't store the vertice indices, because the indices is just iota $[0, N)$.
 *
 */
class Vertices final : public ISimplices
{
  public:
    Vertices() = default;
    /**
     * @brief Get the const view of the vertices, this method generates no data clone.
     * 
     * @return span<const IndexT> 
     */
    [[nodiscard]] span<const IndexT> view() const;
    /**
     * @brief Get the non-const view of the vertices, this method may potentially generate data clone.
     *  
     * @return span<IndexT> 
     */
    [[nodiscard]] span<IndexT> view();

  protected:
    virtual IndexT           get_dim() const override;
    virtual SizeT            get_size() const override;
    virtual void             do_resize(SizeT N) override;
    virtual void             do_clear() override;
    virtual S<ITopoElements> do_clone() const override;
    virtual void             do_reserve(SizeT N) override;

  private:
    size_t                 m_size = 0;
    mutable vector<IndexT> m_simplices;
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
     * @return span<const Vector<IndexT, N + 1>> 
     */
    [[nodiscard]] span<const Vector<IndexT, N + 1>> view() const;
    /**
     * @brief Get the non-const view of the simplices, this method may potentially generate data clone.
     * 
     * @return span<Vector<IndexT, N + 1>> 
     */
    [[nodiscard]] span<Vector<IndexT, N + 1>> view();

  private:
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
 * @brief A collection of edges, $E=\{(i,j) \mid i,j\in V, i\neq j\}$, where $V$ is the set of vertices.
 */
using Edges = Simplices<1>;
/**
 * @brief A collection of triangles, $F=\{(i,j,k) \mid i,j,k\in V, i\neq j\neq k\}$, where $V$ is the set of vertices.
 */
using Triangles = Simplices<2>;
/**
 * @brief A collection of tetrahedra, $T=\{(i,j,k,l) \mid i,j,k,l\in V, i\neq j\neq k\neq l\}$, where $V$ is the set of vertices.
 */
using Tetrahedra = Simplices<3>;
}  // namespace uipc::geometry

#include "details/simplices.inl"