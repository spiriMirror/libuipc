#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/span.h>

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
    [[nodiscard]] SizeT            tuple_size() const;
    /**
     * @brief Get the number of indices in the i-th tuple.
     * 
     * @param IndexT i
     * @return The number of indices in the i-th tuple
     */
    [[nodiscard]] SizeT            tuple_size(IndexT i) const;
    /**
     * @brief Get the number of tuples.
     * 
     * @return The number of tuples
     */
    [[nodiscard]] SizeT            size() const;
    void                           resize(SizeT N);
    void                           clear();
    void                           reserve(SizeT N);
    [[nodiscard]] S<ITopoElements> clone() const;

  protected:
    virtual SizeT            get_tuple_size() const         = 0;
    virtual SizeT            get_tuple_size(IndexT i) const = 0;
    virtual SizeT            get_size() const               = 0;
    virtual void             do_resize(SizeT N)             = 0;
    virtual void             do_clear()                     = 0;
    virtual S<ITopoElements> do_clone() const               = 0;
    virtual void             do_reserve(SizeT N)            = 0;
};
}  // namespace uipc::geometry
