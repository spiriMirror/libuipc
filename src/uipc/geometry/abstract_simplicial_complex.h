#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/simplices.h>

namespace uipc::geometry
{
class AbstractSimplicialComplex;
class ISimplexSlot
{
  public:
    ISimplexSlot()          = default;
    virtual ~ISimplexSlot() = default;

    bool  is_shared() const;
    SizeT size() const;

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

class VertexSlot : public ISimplexSlot
{
    friend class AbstractSimplicialComplex;

  public:
    VertexSlot(S<Vertices> vertices);

    Vertices*       operator->();
    const Vertices* operator->() const;

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

    Simplices<N>*       operator->();
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

class AbstractSimplicialComplex
{
    friend class SimplicialComplex;

  public:
    AbstractSimplicialComplex();

    AbstractSimplicialComplex(const AbstractSimplicialComplex&);
    AbstractSimplicialComplex& operator=(const AbstractSimplicialComplex&);
    AbstractSimplicialComplex(AbstractSimplicialComplex&&) noexcept;
    AbstractSimplicialComplex& operator=(AbstractSimplicialComplex&&) noexcept;

    VertexSlot&       vertices();
    const VertexSlot& vertices() const;

    EdgeSlot&       edges();
    const EdgeSlot& edges() const;

    TriangleSlot&       triangles();
    const TriangleSlot& triangles() const;

    TetrahedronSlot&       tetrahedra();
    const TetrahedronSlot& tetrahedra() const;

  private:
    VertexSlot      m_vertices;
    EdgeSlot        m_edges;
    TriangleSlot    m_triangles;
    TetrahedronSlot m_tetrahedra;
};
}  // namespace uipc::geometry

#include "details/abstract_simplicial_complex.inl"