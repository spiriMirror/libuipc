#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <vector>
#include <span>

namespace uipc::geometry
{
/**
 * @brief An abstract class for storing topological elements. They may be vertices, edges, triangles, tetrahedra, quads or any other elements.
 */
class ITopoElements
{
  public:
    /**
     * .
     * 
     * \return the number of indices in a tuple
     */
    [[nodiscard]] SizeT            tuple_size() const;
    [[nodiscard]] SizeT            tuple_size(IndexT i) const;
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
