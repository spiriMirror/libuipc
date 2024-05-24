#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/simplices.h>

namespace uipc::geometry
{
class AbstractSimplicialComplex;
/**
 * @brief An abstract class representing a simplex slot in an abstract simplicial complex.
 * 
 */
class ISimplexSlot
{
  public:
    ISimplexSlot()          = default;
    virtual ~ISimplexSlot() = default;

    /**
     * @brief Check if the underlying simplices is shared.
     * 
     * @return true, if the simplices is shared
     * @return false, if the simplices is owned
     */
    [[nodiscard]] bool  is_shared() const;
    /**
     * @brief Get the size of the simplices.
     * 
     * @return the size of the simplices
     */
    [[nodiscard]] SizeT size() const;

    // delete copy
    ISimplexSlot(const ISimplexSlot&)            = delete;
    ISimplexSlot& operator=(const ISimplexSlot&) = delete;
    // enable move
    ISimplexSlot(ISimplexSlot&&) noexcept            = default;
    ISimplexSlot& operator=(ISimplexSlot&&) noexcept = default;

  protected:
    friend class AbstractSimplicialComplex;

    void         make_owned();
    virtual void do_make_owned() = 0;

    SizeT         use_count() const;
    virtual SizeT get_use_count() const = 0;

    U<ISimplexSlot>         clone() const;
    virtual U<ISimplexSlot> do_clone() const = 0;

    virtual ISimplices&       simplices();
    virtual ISimplices&       get_simplices() = 0;
    virtual const ISimplices& simplices() const;
    virtual const ISimplices& get_simplices() const = 0;

    ISimplices*       operator->();
    const ISimplices* operator->() const;
};

/**
 * @brief A slot for vertices in an abstract simplicial complex.
 */
class VertexSlot : public ISimplexSlot
{
    friend class AbstractSimplicialComplex;

  public:
    VertexSlot(S<Vertices> vertices);

    /**
     * @brief Get the non-const underlying vertices, this method may potentially generate data clone.
     * 
     * @return non-const underlying vertices 
     */
    Vertices*       operator->();

    /**
     * @brief  Get the const underlying vertices, this method generates no data clone.
     * 
     * @return const underlying vertices 
     */
    const Vertices* operator->() const;

  protected:
    U<VertexSlot> clone() const;

    SizeT             get_use_count() const override;
    U<ISimplexSlot>   do_clone() const override;
    void              do_make_owned() override;
    ISimplices&       get_simplices() override;
    const ISimplices& get_simplices() const override;

  private:
    S<Vertices> m_simplices;
};

template <IndexT N>
class SimplexSlot : public ISimplexSlot
{
    friend class AbstractSimplicialComplex;

  public:
    SimplexSlot(S<Simplices<N>> simplices);

    /**
     * @brief Get the non-const underlying simplices, this method may potentially generate data clone.
     * 
     * @return non-const underlying simplices 
     */
    Simplices<N>*       operator->();
    /**
     * @brief Get the underlying const simplices, this method generates no data clone.
     * 
     * @return const underlying simplices 
     */
    const Simplices<N>* operator->() const;

  protected:
    U<SimplexSlot<N>> clone() const;
    SizeT             get_use_count() const override;
    U<ISimplexSlot>   do_clone() const override;
    void              do_make_owned() override;
    ISimplices&       get_simplices() override;
    const ISimplices& get_simplices() const override;

  private:
    S<Simplices<N>> m_simplices;
};

using EdgeSlot        = SimplexSlot<1>;
using TriangleSlot    = SimplexSlot<2>;
using TetrahedronSlot = SimplexSlot<3>;

/**
 * @brief Represents an abstract simplicial complex, containing vertices, edges, triangles, and tetrahedra.
 * 
 * @note Abstract simplicial complex does not contain any geometric information, such as coordinates of vertices.
 */
class AbstractSimplicialComplex
{
    friend class SimplicialComplex;

  public:
    AbstractSimplicialComplex();

    AbstractSimplicialComplex(const AbstractSimplicialComplex&);
    AbstractSimplicialComplex& operator=(const AbstractSimplicialComplex&);
    AbstractSimplicialComplex(AbstractSimplicialComplex&&) noexcept;
    AbstractSimplicialComplex& operator=(AbstractSimplicialComplex&&) noexcept;

    /**
     * @brief Get the non-const slot for vertices.
     * 
     * @return a non-const slot for vertices
     */
    VertexSlot&       vertices();
    /**
     * @brief Get the const slot for vertices.
     * 
     * @return a const slot for vertices
     */
    const VertexSlot& vertices() const;
    /**
     * @brief Get the non-const slot for edges.
     * 
     * @return a non-const slot for edges
     */
    EdgeSlot&       edges();
    /**
     * @brief Get the const slot for edges.
     * 
     * @return a const slot for edges
     */
    const EdgeSlot& edges() const;
    /**
     * @brief Get the non-const slot for triangles.
     * 
     * @return a non-const slot for triangles
     */
    TriangleSlot&       triangles();
    /**
     * @brief Get the const slot for triangles.
     * 
     * @return a const slot for triangles
     */
    const TriangleSlot& triangles() const;
    /**
     * @brief Get the non-const slot for tetrahedra.
     * 
     * @return a non-const slot for tetrahedra
     */
    TetrahedronSlot&       tetrahedra();
    /**
     * @brief Get the const slot for tetrahedra.
     * 
     * @return a const slot for tetrahedra
     */
    const TetrahedronSlot& tetrahedra() const;

  private:
    VertexSlot      m_vertices;
    EdgeSlot        m_edges;
    TriangleSlot    m_triangles;
    TetrahedronSlot m_tetrahedra;
};
}  // namespace uipc::geometry

#include "details/abstract_simplicial_complex.inl"