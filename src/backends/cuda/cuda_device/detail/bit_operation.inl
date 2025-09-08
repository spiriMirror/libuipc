__device__  __forceinline__ int WARP_BALLOT(int predicate, unsigned int member_mask)
{
    return __ballot_sync(member_mask, predicate);
}