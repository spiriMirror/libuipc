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
template <typename InputIt, typename OutputIt, typename OutputCountIt>
    requires requires(InputIt in_first, InputIt in_last, OutputIt out_unique, OutputCountIt out_counts) {
        // able to assign value from input to output
        *out_unique = *in_first;
        // out_counts must be a iterator to integral type
        std::integral<typename std::iterator_traits<OutputCountIt>::value_type>;
        // able to assign value to out_counts
        *out_counts = 0;
    }
std::size_t run_length_encode(InputIt in_first, InputIt in_last, OutputIt out_unique, OutputCountIt out_counts)
{
    if(in_first == in_last) // empty input
    {
        return 0;
    }

    auto in_current        = in_first;
    auto out_current       = out_unique;
    auto out_count_current = out_counts;

    auto        current_value = *in_current;
    std::size_t current_count = 1;
    std::size_t unique_count  = 1;

    for(++in_current; in_current != in_last; ++in_current)
    {
        if(*in_current == current_value)
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
}  // namespace uipc
