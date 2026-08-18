#pragma once
#include <cinttypes>

namespace muda
{
enum class LoggerBasicType : uint16_t
{
    None,
    Int8,
    Int16,
    Int,
    Int32 = Int,
    Int64,
    Long,
    LongLong,

    UInt8,
    UInt16,
    UInt,
    UInt32 = UInt,
    UInt64,
    ULong,
    ULongLong,

    Float,
    Double,
    String,
    FmtString,


    Object,  // user defined object
};

using LoggerFmtArg = void (*)(void* formatter, const void* obj);

namespace details
{
    class LoggerMetaData
    {
      public:
        LoggerBasicType type     = LoggerBasicType::None;
        uint16_t        exceeded = 0;  // false
        uint32_t        id       = ~0;
        uint32_t        size     = ~0;
        uint32_t        offset   = ~0;
        LoggerFmtArg    fmt_arg  = nullptr;
    };

    class LoggerOffset
    {
      public:
        uint32_t log_id           = 0;
        uint32_t meta_data_offset = 0;
        uint32_t exceed_meta_data = 0;  // false
        uint32_t buffer_offset    = 0;
        uint32_t exceed_buffer    = 0;  // false
    };
}  // namespace details
}  // namespace muda