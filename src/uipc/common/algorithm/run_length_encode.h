#pragma once
#include <concepts>
#include <iterator>

namespace uipc
{
/**
 * @brief Run-length encode the input range, the input range must be sorted
 * 
 * @param in_first Input iterator to the beginning of the input range
 * @param in_last Input iterator to the end of the input range
 * @param out_unique Output iterator to the beginning of the unique values
 * @param out_counts Output iterator to the beginning of the counts of the unique values
 * 
 * @return std::size_t The number of unique values
 */
template <typename InputIt, typename OutputIt, typename OutputCountIt, typename Pred>
    requires requires(InputIt in_first, InputIt in_last, OutputIt out_unique, OutputCountIt out_counts, Pred p) {
        // able to assign value from input to output
        *out_unique = *in_first;
        // out_counts must be a iterator to integral type
        std::integral<typename std::iterator_traits<OutputCountIt>::value_type>;
        // able to assign value to out_counts
        *out_counts = 0;
        // able to compare two values
        {
            p(*in_first, *in_first)
        } -> std::convertible_to<bool>;
    }
std::size_t run_length_encode(InputIt       in_first,
                              InputIt       in_last,
                              OutputIt      out_unique,
                              OutputCountIt out_counts,
                              Pred&&        pred)
{
    if(in_first == in_last)  // empty input
    {
        return 0ull;
    }

    auto in_current        = in_first;
    auto out_current       = out_unique;
    auto out_count_current = out_counts;

    auto        current_value = *in_current;
    std::size_t current_count = 1;
    std::size_t unique_count  = 1;

    for(++in_current; in_current != in_last; ++in_current)
    {
        if(pred(*in_current, current_value))
        {
            ++current_count;
        }
        else
        {
            *out_current       = current_value;
            *out_count_current = current_count;

            ++out_current;
            ++out_count_current;

            current_value = *in_current;
            current_count = 1;
            ++unique_count;
        }
    }

    *out_current       = current_value;
    *out_count_current = current_count;

    return unique_count;
}

template <typename InputIt, typename OutputIt, typename OutputCountIt>
auto run_length_encode(InputIt in_first, InputIt in_last, OutputIt out_unique, OutputCountIt out_counts)
{
    return run_length_encode(in_first, in_last, out_unique, out_counts, std::equal_to<>{});
}


template <typename RandIt, typename OffsetCountIt, typename Pred>
size_t encode_offset_count(RandIt first, RandIt last, OffsetCountIt offsets, OffsetCountIt counts, Pred&& pred)
{
    if(first == last)
    {
        return 0;
    }

    auto current        = first;
    auto offset_current = offsets;
    auto count_current  = counts;

    auto        current_value = *current;
    std::size_t current_count = 1;
    std::size_t unique_count  = 1;
    *offset_current           = 0;

    for(++current; current != last; ++current)
    {
        if(pred(*current, current_value))
        {
            ++current_count;
        }
        else
        {
            *offset_current = current - first;
            *count_current  = current_count;

            ++offset_current;
            ++count_current;

            current_value = *current;
            current_count = 1;
            ++unique_count;
        }
    }

    *count_current = current_count;
    return unique_count;
}

template <typename RandIt, typename OffsetCountIt>
auto encode_offset_count(RandIt first, RandIt last, OffsetCountIt offset, OffsetCountIt count)
{
    return encode_offset_count(first, last, offset, count, std::equal_to<>{});
}
}  // namespace uipc
