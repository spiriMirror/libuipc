#pragma once
#include <muda/ext/field/sub_field_interface.h>

namespace muda
{
template <>
class SubFieldImpl<FieldEntryLayout::AoS> : public SubFieldInterface
{
    friend class SubField;

  protected:
    virtual void build_impl() override;

    virtual size_t require_total_buffer_byte_size(size_t element_count) override;

    virtual void calculate_new_cores(std::byte*           byte_buffer,
                                     size_t               total_bytes,
                                     size_t               element_count,
                                     span<FieldEntryCore> new_cores) override
    {
        // no need to implement anything
    }

  public:
    using SubFieldInterface::SubFieldInterface;
    virtual ~SubFieldImpl() override = default;
};
}  // namespace muda

#include "details/aos_sub_field.inl"