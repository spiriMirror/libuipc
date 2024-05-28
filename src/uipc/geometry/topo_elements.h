#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/span.h>
#include <uipc/backend/buffer_view.h>

namespace uipc::geometry
{
/**
 * @brief An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements.
 */
class ITopoElements
{
  public:
    /**
     * @brief Get the number of indices in a tuple.
     * 
     * If return ~0ull, it means the number of indices in a tuple is not fixed. user should call tuple_size(IndexT i) to get the number of indices in a tuple.
     * 
     * \return The number of indices in a tuple
     */
    [[nodiscard]] SizeT tuple_size() const noexcept;
    /**
     * @brief Get the number of indices in the i-th tuple.
     * 
     * @param IndexT i
     * @return The number of indices in the i-th tuple
     */
    [[nodiscard]] SizeT tuple_size(IndexT i) const noexcept;
    /**
     * @brief Get the number of tuples.
     * 
     * @return The number of tuples
     */
    [[nodiscard]] SizeT size() const noexcept;
    /**
     * @brief Resize the number of tuples.
     * 
     * @param SizeT N
     */
    void resize(SizeT N);
    /**
     * @brief Clear the topology.
     */
    void clear();
    /**
     * @brief Reserve the memory for the topology.
     * 
     * @param SizeT N
     */
    void reserve(SizeT N);
    /**
     * @brief Clone the topology.
     * 
     * @return A shared pointer to the cloned topology
     */
    [[nodiscard]] S<ITopoElements> clone() const;

    friend backend::BufferView backend_view(const ITopoElements& simplices) noexcept;

  protected:
    virtual backend::BufferView get_backend_view() const noexcept       = 0;
    virtual SizeT               get_tuple_size() const noexcept         = 0;
    virtual SizeT               get_tuple_size(IndexT i) const noexcept = 0;
    virtual SizeT               get_size() const noexcept               = 0;
    virtual void                do_resize(SizeT N)                      = 0;
    virtual void                do_clear()                              = 0;
    virtual S<ITopoElements>    do_clone() const                        = 0;
    virtual void                do_reserve(SizeT N)                     = 0;

  private:
    backend::BufferView backend_view() const noexcept;
};
}  // namespace uipc::geometry
