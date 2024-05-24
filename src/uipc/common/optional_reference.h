#pragma once
#include <optional>
#include <boost/optional.hpp>

namespace uipc
{
/**
 * @brief An empty reference class, represent an empty reference for OptionalRef.
 */
class EmptyRef
{
};

/**
 * @brief An optional reference class, disable several operators of the boost::optional.
 * 
 * - Never allow any assignment or copy operation.
 * - Apply const propagation.
 *
 * @tparam T A const or non-const ValueType.
 */
template <typename T>
    requires(!std::is_reference_v<T>)
class OptionalRef : private boost::optional<std::add_lvalue_reference_t<T>>
{
  private:
    using Base = boost::optional<std::add_lvalue_reference_t<T>>;

    static constexpr bool is_const = std::is_const_v<T>;

    using NonConstValue = std::remove_const_t<T>;
    using ConstValue    = std::add_const_t<T>;
    using Value         = T;

    using NonConstRef = std::add_lvalue_reference_t<NonConstValue>;
    using ConstRef    = std::add_lvalue_reference_t<ConstValue>;
    using Ref         = std::add_lvalue_reference_t<T>;

    using NonConstPointer = std::add_pointer_t<NonConstValue>;
    using ConstPointer    = std::add_pointer_t<ConstValue>;
    using Pointer         = std::add_pointer_t<T>;

    using ConstOptionalRef = OptionalRef<ConstValue>;

  public:
    using Base::Base;
    using Base::operator bool;
    using Base::operator==;
    using Base::operator!;
    
    // delete copy constructor
    OptionalRef(const OptionalRef&) = delete;
    OptionalRef& operator=(const OptionalRef&) = delete;

    OptionalRef(EmptyRef) { Base::reset(); }

    Ref      operator*() { return Base::operator*(); }
    ConstRef operator*() const { return Base::operator*(); }

    Pointer      operator->() { return Base::operator->(); }
    ConstPointer operator->() const { return Base::operator->(); }

    Ref      value() { return Base::value(); }
    ConstRef value() const { return Base::value(); }

    Pointer      get_ptr() { return Base::get_ptr(); }
    ConstPointer get_ptr() const { return Base::get_ptr(); }
};


}  // namespace uipc
