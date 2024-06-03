#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/simplices.h>

namespace uipc::geometry
{
class AbstractSimplicialComplex;
/**
 * @brief An abstract class representing a simplex slot in an abstract simplicial complex.
 */
class ISimplexSlot
{
  public:
    ISimplexSlot()          = default;
    virtual ~ISimplexSlot() = default;

    // delete copy
    ISimplexSlot(const ISimplexSlot&)            = delete;
    ISimplexSlot& operator=(const ISimplexSlot&) = delete;
    // enable move
    ISimplexSlot(ISimplexSlot&&) noexcept            = default;
    ISimplexSlot& operator=(ISimplexSlot&&) noexcept = default;

    /**
     * @brief Check if the underlying simplices is shared.
     * 
     * @return true, if the simplices is shared
     * @return false, if the simplices is owned
     */
    [[nodiscard]] bool is_shared() const;
    /**
     * @brief Get the size of the simplices.
     * 
     * @return the size of the simplices
     */
    [[nodiscard]] SizeT size() const;

    void share(const ISimplexSlot& other);

    friend backend::BufferView backend_view(const ISimplexSlot&) noexcept;

    void reorder(span<const SizeT> O);
    void resize(SizeT size);
    void reserve(SizeT capacity);
    void clear();

  protected:
    friend class AbstractSimplicialComplex;

    void         make_owned();
    virtual void do_make_owned() = 0;

    SizeT         use_count() const noexcept;
    virtual SizeT get_use_count() const noexcept = 0;

    S<ISimplexSlot>         clone() const;
    virtual S<ISimplexSlot> do_clone() const = 0;

    virtual ISimplices& simplices() noexcept;
    virtual ISimplices& get_simplices() noexcept = 0;

    virtual const ISimplices& simplices() const noexcept;
    virtual const ISimplices& get_simplices() const noexcept = 0;

    virtual void do_reorder(span<const SizeT> O) noexcept = 0;
    virtual void do_resize(SizeT size)                    = 0;
    virtual void do_reserve(SizeT capacity)               = 0;
    virtual void do_clear()                               = 0;
    virtual void do_share(const ISimplexSlot& other)      = 0;
};

/**
 * @brief A slot for vertices in an abstract simplicial complex.
 */
class VertexSlot : public ISimplexSlot
{
    friend class AbstractSimplicialComplex;

  public:
    static constexpr IndexT Dimension = 0;
    using ValueT                      = IndexT;

    VertexSlot(S<Vertices> vertices) noexcept;

    /**
     * @brief Get a non-const view of the vertices.
     * 
     * @param slot the slot for vertices
     * @return A non-const view of the vertices
     */
    friend span<IndexT> view(VertexSlot& slot);

    /**
     * @brief Get a const view of the vertices.
     * 
     * @return A const view of the vertices
     */
    span<const IndexT> view() const;

  protected:
    S<VertexSlot> clone() const;

    virtual SizeT             get_use_count() const noexcept override;
    virtual ISimplices&       get_simplices() noexcept override;
    virtual const ISimplices& get_simplices() const noexcept override;

    virtual S<ISimplexSlot> do_clone() const override;
    virtual void            do_make_owned() override;

    virtual void do_reorder(span<const SizeT> O) noexcept override;
    virtual void do_resize(SizeT size) override;
    virtual void do_reserve(SizeT capacity) override;
    virtual void do_clear() override;
    virtual void do_share(const ISimplexSlot& other) override;

  private:
    S<Vertices> m_simplices;
};

span<IndexT> view(VertexSlot& slot);

template <IndexT N>
class SimplexSlot : public ISimplexSlot
{
    friend class AbstractSimplicialComplex;

  public:
    static constexpr IndexT Dimension = N;
    using ValueT                      = Vector<IndexT, N + 1>;

    SimplexSlot(S<Simplices<N>> simplices) noexcept;

    /**
     * @brief Get a non-const view of the simplices.
     * 
     * @tparam M dimension of the simplices
     * @param slot the slot for simplices
     * @return A non-const view of the simplices
     */
    template <IndexT M>
    friend span<typename SimplexSlot<M>::ValueT> view(SimplexSlot<M>& slot);


    /**
     * @brief Get a const view of the simplices.
     * 
     * @return A const view of the simplices
     */
    span<const ValueT> view() const noexcept;

  protected:
    S<SimplexSlot<N>>         clone() const;
    virtual SizeT             get_use_count() const noexcept override;
    virtual ISimplices&       get_simplices() noexcept override;
    virtual const ISimplices& get_simplices() const noexcept override;

    virtual S<ISimplexSlot> do_clone() const override;
    virtual void            do_make_owned() override;

    virtual void do_reorder(span<const SizeT> O) noexcept override;
    virtual void do_resize(SizeT size) override;
    virtual void do_reserve(SizeT capacity) override;
    virtual void do_clear() override;
    virtual void do_share(const ISimplexSlot& other) override;

  private:
    S<Simplices<N>> m_simplices;
};

/**
 * @brief Alias for a slot for edges in an abstract simplicial complex.
 */
using EdgeSlot = SimplexSlot<1>;

/**
 * @brief Alias for a slot for triangles in an abstract simplicial complex.
 */
using TriangleSlot = SimplexSlot<2>;

/**
 * @brief Alias for a slot for tetrahedra in an abstract simplicial complex.
 */
using TetrahedronSlot = SimplexSlot<3>;
}  // namespace uipc::geometry

#include "details/simplex_slot.inl"
