#pragma once
#include <cuda_runtime.h>
/**
 * @brief CUDA Intrinsics Header File
 * 
 * This header provides declarations for CUDA intrinsics to enable
 * IntelliSense support in Visual Studio and other IDEs.
 * 
 * When using IntelliSense (__INTELLISENSE__ is defined), these are function
 * declarations. During actual compilation, CUDA provides the real implementations.
 * 
 * Based on:
 *   - CUDA Math API Reference Manual 13.0:
 *     - Integer Intrinsics (Section 13)
 *     - Type Casting Intrinsics (Section 11)
 *   - CUDA C Programming Guide:
 *     - Warp Shuffle Functions
 *     - Memory Fence Functions (Section 10.5)
 *     - Synchronization Functions (Section 10.6)
 *     - Atomic Functions (Section 10.14)
 */

// Include CUDA headers if available
#ifdef __INTELLISENSE__
// ============================================================================
// Bit Manipulation Functions
// ============================================================================

/**
 * @brief Reverse the bit order of a 32-bit unsigned integer
 * @param x 32-bit unsigned integer
 * @return Bit-reversed value of x
 */
__device__ unsigned int __brev(unsigned int x);

/**
 * @brief Reverse the bit order of a 64-bit unsigned integer
 * @param x 64-bit unsigned integer
 * @return Bit-reversed value of x
 */
__device__ unsigned long long int __brevll(unsigned long long int x);

/**
 * @brief Return selected bytes from two 32-bit unsigned integers
 * @param x First 32-bit unsigned integer
 * @param y Second 32-bit unsigned integer
 * @param s Selector specifying which bytes to extract
 * @return 32-bit integer with selected bytes
 */
__device__ unsigned int __byte_perm(unsigned int x, unsigned int y, unsigned int s);

/**
 * @brief Return the number of consecutive high-order zero bits in a 32-bit integer
 * @param x 32-bit integer
 * @return Number of leading zero bits (0-32)
 */
__device__ int __clz(int x);

/**
 * @brief Count the number of consecutive high-order zero bits in a 64-bit integer
 * @param x 64-bit integer
 * @return Number of leading zero bits (0-64)
 */
__device__ int __clzll(long long int x);

/**
 * @brief Find the position of the least significant bit set to 1 in a 32-bit integer
 * @param x 32-bit integer
 * @return Position of first set bit (0-32, 0 if no bits set)
 */
__device__ int __ffs(int x);

/**
 * @brief Find the position of the least significant bit set to 1 in a 64-bit integer
 * @param x 64-bit integer
 * @return Position of first set bit (0-64, 0 if no bits set)
 */
__device__ int __ffsll(long long int x);

/**
 * @brief Find the position of the n-th set to 1 bit in a 32-bit integer
 * @param mask 32-bit mask value
 * @param base Base position (must be <= 31)
 * @param offset Offset for n-th bit
 * @return Position of n-th set bit, or 0xFFFFFFFF if not found
 */
__device__ unsigned __fns(unsigned mask, unsigned base, int offset);

/**
 * @brief Count the number of bits that are set to 1 in a 32-bit integer
 * @param x 32-bit unsigned integer
 * @return Number of set bits (0-32)
 */
__device__ int __popc(unsigned int x);

/**
 * @brief Count the number of bits that are set to 1 in a 64-bit integer
 * @param x 64-bit unsigned integer
 * @return Number of set bits (0-64)
 */
__device__ int __popcll(unsigned long long int x);

// ============================================================================
// Byte Swap Functions (Host and Device)
// ============================================================================

/**
 * @brief Reverse the order of bytes of the 16-bit unsigned integer
 * @param x 16-bit unsigned integer
 * @return x with byte order reversed
 */
__host__ __device__ unsigned short __nv_bswap16(unsigned short x);

/**
 * @brief Reverse the order of bytes of the 32-bit unsigned integer
 * @param x 32-bit unsigned integer
 * @return x with byte order reversed
 */
__host__ __device__ unsigned int __nv_bswap32(unsigned int x);

/**
 * @brief Reverse the order of bytes of the 64-bit unsigned integer
 * @param x 64-bit unsigned integer
 * @return x with byte order reversed
 */
__host__ __device__ unsigned long long __nv_bswap64(unsigned long long x);

// ============================================================================
// Funnel Shift Functions
// ============================================================================

/**
 * @brief Concatenate hi:lo, shift left by shift & 31 bits, return MSB 32 bits
 * @param lo Lower 32 bits
 * @param hi Upper 32 bits
 * @param shift Shift amount
 * @return Most significant 32 bits of shifted 64-bit value
 */
__device__ unsigned int __funnelshift_l(unsigned int lo, unsigned int hi, unsigned int shift);

/**
 * @brief Concatenate hi:lo, shift left by min(shift, 32) bits, return MSB 32 bits
 * @param lo Lower 32 bits
 * @param hi Upper 32 bits
 * @param shift Shift amount
 * @return Most significant 32 bits of shifted 64-bit value
 */
__device__ unsigned int __funnelshift_lc(unsigned int lo, unsigned int hi, unsigned int shift);

/**
 * @brief Concatenate hi:lo, shift right by shift & 31 bits, return LSB 32 bits
 * @param lo Lower 32 bits
 * @param hi Upper 32 bits
 * @param shift Shift amount
 * @return Least significant 32 bits of shifted 64-bit value
 */
__device__ unsigned int __funnelshift_r(unsigned int lo, unsigned int hi, unsigned int shift);

/**
 * @brief Concatenate hi:lo, shift right by min(shift, 32) bits, return LSB 32 bits
 * @param lo Lower 32 bits
 * @param hi Upper 32 bits
 * @param shift Shift amount
 * @return Least significant 32 bits of shifted 64-bit value
 */
__device__ unsigned int __funnelshift_rc(unsigned int lo, unsigned int hi, unsigned int shift);

// ============================================================================
// Arithmetic Functions
// ============================================================================

/**
 * @brief Compute average of signed input arguments, avoiding overflow
 * @param x First signed integer
 * @param y Second signed integer
 * @return (x + y) >> 1
 */
__device__ int __hadd(int x, int y);

/**
 * @brief Compute average of unsigned input arguments, avoiding overflow
 * @param x First unsigned integer
 * @param y Second unsigned integer
 * @return (x + y) >> 1
 */
__device__ unsigned int __uhadd(unsigned int x, unsigned int y);

/**
 * @brief Compute rounded average of signed input arguments, avoiding overflow
 * @param x First signed integer
 * @param y Second signed integer
 * @return (x + y + 1) >> 1
 */
__device__ int __rhadd(int x, int y);

/**
 * @brief Compute rounded average of unsigned input arguments, avoiding overflow
 * @param x First unsigned integer
 * @param y Second unsigned integer
 * @return (x + y + 1) >> 1
 */
__device__ unsigned int __urhadd(unsigned int x, unsigned int y);

/**
 * @brief Calculate the least significant 32 bits of the product of the least significant 24 bits
 * @param x First integer (only 24 LSB used)
 * @param y Second integer (only 24 LSB used)
 * @return Least significant 32 bits of product
 */
__device__ int __mul24(int x, int y);

/**
 * @brief Calculate the least significant 32 bits of the product of the least significant 24 bits
 * @param x First unsigned integer (only 24 LSB used)
 * @param y Second unsigned integer (only 24 LSB used)
 * @return Least significant 32 bits of product
 */
__device__ unsigned int __umul24(unsigned int x, unsigned int y);

/**
 * @brief Calculate the most significant 32 bits of the product of two 32-bit integers
 * @param x First 32-bit integer
 * @param y Second 32-bit integer
 * @return Most significant 32 bits of 64-bit product
 */
__device__ int __mulhi(int x, int y);

/**
 * @brief Calculate the most significant 32 bits of the product of two 32-bit unsigned integers
 * @param x First 32-bit unsigned integer
 * @param y Second 32-bit unsigned integer
 * @return Most significant 32 bits of 64-bit product
 */
__device__ unsigned int __umulhi(unsigned int x, unsigned int y);

/**
 * @brief Calculate the most significant 64 bits of the product of two 64-bit integers
 * @param x First 64-bit integer
 * @param y Second 64-bit integer
 * @return Most significant 64 bits of 128-bit product
 */
__device__ long long int __mul64hi(long long int x, long long int y);

/**
 * @brief Calculate the most significant 64 bits of the product of two 64-bit unsigned integers
 * @param x First 64-bit unsigned integer
 * @param y Second 64-bit unsigned integer
 * @return Most significant 64 bits of 128-bit product
 */
__device__ unsigned long long int __umul64hi(unsigned long long int x,
                                             unsigned long long int y);

/**
 * @brief Calculate |x - y| + z (sum of absolute difference) for signed integers
 * @param x First signed 32-bit integer
 * @param y Second signed 32-bit integer
 * @param z Unsigned 32-bit integer
 * @return |x - y| + z
 */
__device__ unsigned int __sad(int x, int y, unsigned int z);

/**
 * @brief Calculate |x - y| + z (sum of absolute difference) for unsigned integers
 * @param x First unsigned 32-bit integer
 * @param y Second unsigned 32-bit integer
 * @param z Third unsigned 32-bit integer
 * @return |x - y| + z
 */
__device__ unsigned int __usad(unsigned int x, unsigned int y, unsigned int z);

// ============================================================================
// Dot Product Functions (DP2A)
// ============================================================================

/**
 * @brief Two-way signed int16 by int8 dot product with int32 accumulate (upper half)
 * @param srcA Source A containing two packed 16-bit integers
 * @param srcB Source B containing packed 8-bit integers (upper 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ int __dp2a_hi(int srcA, int srcB, int c);

/**
 * @brief Two-way unsigned int16 by int8 dot product with unsigned int32 accumulate (upper half)
 * @param srcA Source A containing two packed 16-bit integers
 * @param srcB Source B containing packed 8-bit integers (upper 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ unsigned int __dp2a_hi(unsigned int srcA, unsigned int srcB, unsigned int c);

/**
 * @brief Two-way unsigned int16 by int8 dot product with unsigned int32 accumulate (upper half)
 * @param srcA Source A vector with two packed 16-bit integers
 * @param srcB Source B vector with packed 8-bit integers (upper 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ unsigned int __dp2a_hi(ushort2 srcA, uchar4 srcB, unsigned int c);

/**
 * @brief Two-way signed int16 by int8 dot product with int32 accumulate (upper half)
 * @param srcA Source A vector with two packed 16-bit integers
 * @param srcB Source B vector with packed 8-bit integers (upper 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ int __dp2a_hi(short2 srcA, char4 srcB, int c);

/**
 * @brief Two-way signed int16 by int8 dot product with int32 accumulate (lower half)
 * @param srcA Source A containing two packed 16-bit integers
 * @param srcB Source B containing packed 8-bit integers (lower 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ int __dp2a_lo(int srcA, int srcB, int c);

/**
 * @brief Two-way unsigned int16 by int8 dot product with unsigned int32 accumulate (lower half)
 * @param srcA Source A containing two packed 16-bit integers
 * @param srcB Source B containing packed 8-bit integers (lower 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ unsigned int __dp2a_lo(unsigned int srcA, unsigned int srcB, unsigned int c);

/**
 * @brief Two-way unsigned int16 by int8 dot product with unsigned int32 accumulate (lower half)
 * @param srcA Source A vector with two packed 16-bit integers
 * @param srcB Source B vector with packed 8-bit integers (lower 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ unsigned int __dp2a_lo(ushort2 srcA, uchar4 srcB, unsigned int c);

/**
 * @brief Two-way signed int16 by int8 dot product with int32 accumulate (lower half)
 * @param srcA Source A vector with two packed 16-bit integers
 * @param srcB Source B vector with packed 8-bit integers (lower 16 bits used)
 * @param c Accumulator
 * @return Dot product result
 */
__device__ int __dp2a_lo(short2 srcA, char4 srcB, int c);

// ============================================================================
// Dot Product Functions (DP4A)
// ============================================================================

/**
 * @brief Four-way unsigned int8 dot product with unsigned int32 accumulate
 * @param srcA Source A vector with four packed 8-bit integers
 * @param srcB Source B vector with four packed 8-bit integers
 * @param c Accumulator
 * @return Dot product result
 */
__device__ unsigned int __dp4a(uchar4 srcA, uchar4 srcB, unsigned int c);

/**
 * @brief Four-way unsigned int8 dot product with unsigned int32 accumulate
 * @param srcA Source A containing four packed 8-bit integers
 * @param srcB Source B containing four packed 8-bit integers
 * @param c Accumulator
 * @return Dot product result
 */
__device__ unsigned int __dp4a(unsigned int srcA, unsigned int srcB, unsigned int c);

/**
 * @brief Four-way signed int8 dot product with int32 accumulate
 * @param srcA Source A containing four packed 8-bit integers
 * @param srcB Source B containing four packed 8-bit integers
 * @param c Accumulator
 * @return Dot product result
 */
__device__ int __dp4a(int srcA, int srcB, int c);

/**
 * @brief Four-way signed int8 dot product with int32 accumulate
 * @param srcA Source A vector with four packed 8-bit integers
 * @param srcB Source B vector with four packed 8-bit integers
 * @param c Accumulator
 * @return Dot product result
 */
__device__ int __dp4a(char4 srcA, char4 srcB, int c);

// ============================================================================
// Type Casting Functions
// ============================================================================

// Double to Float Conversions
/**
 * @brief Convert a double to a float in round-down mode
 * @param x Double-precision floating-point value
 * @return Converted single-precision floating-point value
 */
__device__ float __double2float_rd(double x);

/**
 * @brief Convert a double to a float in round-to-nearest-even mode
 * @param x Double-precision floating-point value
 * @return Converted single-precision floating-point value
 */
__device__ float __double2float_rn(double x);

/**
 * @brief Convert a double to a float in round-up mode
 * @param x Double-precision floating-point value
 * @return Converted single-precision floating-point value
 */
__device__ float __double2float_ru(double x);

/**
 * @brief Convert a double to a float in round-towards-zero mode
 * @param x Double-precision floating-point value
 * @return Converted single-precision floating-point value
 */
__device__ float __double2float_rz(double x);

// Double to Integer Conversions
/**
 * @brief Convert a double to a signed int in round-down mode
 * @param x Double-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __double2int_rd(double x);

/**
 * @brief Convert a double to a signed int in round-to-nearest-even mode
 * @param x Double-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __double2int_rn(double x);

/**
 * @brief Convert a double to a signed int in round-up mode
 * @param x Double-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __double2int_ru(double x);

/**
 * @brief Convert a double to a signed int in round-towards-zero mode
 * @param x Double-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __double2int_rz(double x);

/**
 * @brief Convert a double to a signed 64-bit int in round-down mode
 * @param x Double-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __double2ll_rd(double x);

/**
 * @brief Convert a double to a signed 64-bit int in round-to-nearest-even mode
 * @param x Double-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __double2ll_rn(double x);

/**
 * @brief Convert a double to a signed 64-bit int in round-up mode
 * @param x Double-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __double2ll_ru(double x);

/**
 * @brief Convert a double to a signed 64-bit int in round-towards-zero mode
 * @param x Double-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __double2ll_rz(double x);

/**
 * @brief Convert a double to an unsigned int in round-down mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __double2uint_rd(double x);

/**
 * @brief Convert a double to an unsigned int in round-to-nearest-even mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __double2uint_rn(double x);

/**
 * @brief Convert a double to an unsigned int in round-up mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __double2uint_ru(double x);

/**
 * @brief Convert a double to an unsigned int in round-towards-zero mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __double2uint_rz(double x);

/**
 * @brief Convert a double to an unsigned 64-bit int in round-down mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __double2ull_rd(double x);

/**
 * @brief Convert a double to an unsigned 64-bit int in round-to-nearest-even mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __double2ull_rn(double x);

/**
 * @brief Convert a double to an unsigned 64-bit int in round-up mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __double2ull_ru(double x);

/**
 * @brief Convert a double to an unsigned 64-bit int in round-towards-zero mode
 * @param x Double-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __double2ull_rz(double x);

// Double Bit Reinterpretation
/**
 * @brief Reinterpret high 32 bits in a double as a signed integer
 * @param x Double-precision floating-point value
 * @return High 32 bits as signed integer
 */
__device__ int __double2hiint(double x);

/**
 * @brief Reinterpret low 32 bits in a double as a signed integer
 * @param x Double-precision floating-point value
 * @return Low 32 bits as signed integer
 */
__device__ int __double2loint(double x);

/**
 * @brief Reinterpret bits in a double as a 64-bit signed integer
 * @param x Double-precision floating-point value
 * @return Reinterpreted 64-bit signed integer value
 */
__device__ long long int __double_as_longlong(double x);

/**
 * @brief Reinterpret high and low 32-bit integer values as a double
 * @param hi High 32 bits
 * @param lo Low 32 bits
 * @return Reinterpreted double-precision floating-point value
 */
__device__ double __hiloint2double(int hi, int lo);

// Float to Integer Conversions
/**
 * @brief Convert a float to a signed integer in round-down mode
 * @param x Single-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __float2int_rd(float x);

/**
 * @brief Convert a float to a signed integer in round-to-nearest-even mode
 * @param x Single-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __float2int_rn(float x);

/**
 * @brief Convert a float to a signed integer in round-up mode
 * @param x Single-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __float2int_ru(float x);

/**
 * @brief Convert a float to a signed integer in round-towards-zero mode
 * @param x Single-precision floating-point value
 * @return Converted signed integer value
 */
__device__ int __float2int_rz(float x);

/**
 * @brief Convert a float to a signed 64-bit integer in round-down mode
 * @param x Single-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __float2ll_rd(float x);

/**
 * @brief Convert a float to a signed 64-bit integer in round-to-nearest-even mode
 * @param x Single-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __float2ll_rn(float x);

/**
 * @brief Convert a float to a signed 64-bit integer in round-up mode
 * @param x Single-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __float2ll_ru(float x);

/**
 * @brief Convert a float to a signed 64-bit integer in round-towards-zero mode
 * @param x Single-precision floating-point value
 * @return Converted signed 64-bit integer value
 */
__device__ long long int __float2ll_rz(float x);

/**
 * @brief Convert a float to an unsigned integer in round-down mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __float2uint_rd(float x);

/**
 * @brief Convert a float to an unsigned integer in round-to-nearest-even mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __float2uint_rn(float x);

/**
 * @brief Convert a float to an unsigned integer in round-up mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __float2uint_ru(float x);

/**
 * @brief Convert a float to an unsigned integer in round-towards-zero mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned integer value
 */
__device__ unsigned int __float2uint_rz(float x);

/**
 * @brief Convert a float to an unsigned 64-bit integer in round-down mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __float2ull_rd(float x);

/**
 * @brief Convert a float to an unsigned 64-bit integer in round-to-nearest-even mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __float2ull_rn(float x);

/**
 * @brief Convert a float to an unsigned 64-bit integer in round-up mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __float2ull_ru(float x);

/**
 * @brief Convert a float to an unsigned 64-bit integer in round-towards-zero mode
 * @param x Single-precision floating-point value
 * @return Converted unsigned 64-bit integer value
 */
__device__ unsigned long long int __float2ull_rz(float x);

// Float Bit Reinterpretation
/**
 * @brief Reinterpret bits in a float as a signed integer
 * @param x Single-precision floating-point value
 * @return Reinterpreted signed integer value
 */
__device__ int __float_as_int(float x);

/**
 * @brief Reinterpret bits in a float as an unsigned integer
 * @param x Single-precision floating-point value
 * @return Reinterpreted unsigned integer value
 */
__device__ unsigned int __float_as_uint(float x);

// Integer to Float/Double Conversions
/**
 * @brief Convert a signed integer to a float in round-down mode
 * @param x Signed integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __int2float_rd(int x);

/**
 * @brief Convert a signed integer to a float in round-to-nearest-even mode
 * @param x Signed integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __int2float_rn(int x);

/**
 * @brief Convert a signed integer to a float in round-up mode
 * @param x Signed integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __int2float_ru(int x);

/**
 * @brief Convert a signed integer to a float in round-towards-zero mode
 * @param x Signed integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __int2float_rz(int x);

/**
 * @brief Convert a signed int to a double
 * @param x Signed integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __int2double_rn(int x);

/**
 * @brief Convert an unsigned int to a double
 * @param x Unsigned integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __uint2double_rn(unsigned int x);

/**
 * @brief Convert an unsigned integer to a float in round-down mode
 * @param x Unsigned integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __uint2float_rd(unsigned int x);

/**
 * @brief Convert an unsigned integer to a float in round-to-nearest-even mode
 * @param x Unsigned integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __uint2float_rn(unsigned int x);

/**
 * @brief Convert an unsigned integer to a float in round-up mode
 * @param x Unsigned integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __uint2float_ru(unsigned int x);

/**
 * @brief Convert an unsigned integer to a float in round-towards-zero mode
 * @param x Unsigned integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __uint2float_rz(unsigned int x);

// Integer Bit Reinterpretation
/**
 * @brief Reinterpret bits in an integer as a float
 * @param x Signed integer value
 * @return Reinterpreted single-precision floating-point value
 */
__device__ float __int_as_float(int x);

/**
 * @brief Reinterpret bits in an unsigned integer as a float
 * @param x Unsigned integer value
 * @return Reinterpreted single-precision floating-point value
 */
__device__ float __uint_as_float(unsigned int x);

// Long Long to Float/Double Conversions
/**
 * @brief Convert a signed 64-bit int to a double in round-down mode
 * @param x Signed 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ll2double_rd(long long int x);

/**
 * @brief Convert a signed 64-bit int to a double in round-to-nearest-even mode
 * @param x Signed 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ll2double_rn(long long int x);

/**
 * @brief Convert a signed 64-bit int to a double in round-up mode
 * @param x Signed 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ll2double_ru(long long int x);

/**
 * @brief Convert a signed 64-bit int to a double in round-towards-zero mode
 * @param x Signed 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ll2double_rz(long long int x);

/**
 * @brief Convert a signed integer to a float in round-down mode
 * @param x Signed 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ll2float_rd(long long int x);

/**
 * @brief Convert a signed 64-bit integer to a float in round-to-nearest-even mode
 * @param x Signed 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ll2float_rn(long long int x);

/**
 * @brief Convert a signed integer to a float in round-up mode
 * @param x Signed 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ll2float_ru(long long int x);

/**
 * @brief Convert a signed integer to a float in round-towards-zero mode
 * @param x Signed 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ll2float_rz(long long int x);

/**
 * @brief Convert an unsigned 64-bit int to a double in round-down mode
 * @param x Unsigned 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ull2double_rd(unsigned long long int x);

/**
 * @brief Convert an unsigned 64-bit int to a double in round-to-nearest-even mode
 * @param x Unsigned 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ull2double_rn(unsigned long long int x);

/**
 * @brief Convert an unsigned 64-bit int to a double in round-up mode
 * @param x Unsigned 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ull2double_ru(unsigned long long int x);

/**
 * @brief Convert an unsigned 64-bit int to a double in round-towards-zero mode
 * @param x Unsigned 64-bit integer value
 * @return Converted double-precision floating-point value
 */
__device__ double __ull2double_rz(unsigned long long int x);

/**
 * @brief Convert an unsigned integer to a float in round-down mode
 * @param x Unsigned 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ull2float_rd(unsigned long long int x);

/**
 * @brief Convert an unsigned integer to a float in round-to-nearest-even mode
 * @param x Unsigned 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ull2float_rn(unsigned long long int x);

/**
 * @brief Convert an unsigned integer to a float in round-up mode
 * @param x Unsigned 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ull2float_ru(unsigned long long int x);

/**
 * @brief Convert an unsigned integer to a float in round-towards-zero mode
 * @param x Unsigned 64-bit integer value
 * @return Converted single-precision floating-point value
 */
__device__ float __ull2float_rz(unsigned long long int x);

// Long Long Bit Reinterpretation
/**
 * @brief Reinterpret bits in a 64-bit signed integer as a double
 * @param x 64-bit signed integer value
 * @return Reinterpreted double-precision floating-point value
 */
__device__ double __longlong_as_double(long long int x);

// ============================================================================
// Warp Shuffle Functions
// ============================================================================

// Warp Shuffle - General Shuffle
/**
 * @brief Shuffle a variable across threads in a warp
 * @param mask Member mask for threads participating in the shuffle
 * @param var Variable to shuffle
 * @param srcLane Source lane index (0-31)
 * @param width Width of the shuffle (must be power of 2, <= 32)
 * @return Value from source lane
 */
__device__ int __shfl_sync(unsigned int mask, int var, int srcLane, int width = 32);
__device__ unsigned int __shfl_sync(unsigned int mask, unsigned int var, int srcLane, int width = 32);
__device__ long long int __shfl_sync(unsigned int mask, long long int var, int srcLane, int width = 32);
__device__ unsigned long long int __shfl_sync(unsigned int           mask,
                                              unsigned long long int var,
                                              int                    srcLane,
                                              int width = 32);
__device__ float __shfl_sync(unsigned int mask, float var, int srcLane, int width = 32);
__device__ double __shfl_sync(unsigned int mask, double var, int srcLane, int width = 32);

// Warp Shuffle Up
/**
 * @brief Shuffle a variable up the warp
 * @param mask Member mask for threads participating in the shuffle
 * @param var Variable to shuffle
 * @param delta Offset to shuffle up by
 * @param width Width of the shuffle (must be power of 2, <= 32)
 * @return Value from lane (laneId - delta)
 */
__device__ int __shfl_up_sync(unsigned int mask, int var, unsigned int delta, int width = 32);
__device__ unsigned int           __shfl_up_sync(unsigned int mask,
                                                 unsigned int var,
                                                 unsigned int delta,
                                                 int          width = 32);
__device__ long long int          __shfl_up_sync(unsigned int  mask,
                                                 long long int var,
                                                 unsigned int  delta,
                                                 int           width = 32);
__device__ unsigned long long int __shfl_up_sync(unsigned int           mask,
                                                 unsigned long long int var,
                                                 unsigned int           delta,
                                                 int width = 32);
__device__ float __shfl_up_sync(unsigned int mask, float var, unsigned int delta, int width = 32);
__device__ double __shfl_up_sync(unsigned int mask, double var, unsigned int delta, int width = 32);

// Warp Shuffle Down
/**
 * @brief Shuffle a variable down the warp
 * @param mask Member mask for threads participating in the shuffle
 * @param var Variable to shuffle
 * @param delta Offset to shuffle down by
 * @param width Width of the shuffle (must be power of 2, <= 32)
 * @return Value from lane (laneId + delta)
 */
__device__ int __shfl_down_sync(unsigned int mask, int var, unsigned int delta, int width = 32);
__device__ unsigned int           __shfl_down_sync(unsigned int mask,
                                                   unsigned int var,
                                                   unsigned int delta,
                                                   int          width = 32);
__device__ long long int          __shfl_down_sync(unsigned int  mask,
                                                   long long int var,
                                                   unsigned int  delta,
                                                   int           width = 32);
__device__ unsigned long long int __shfl_down_sync(unsigned int           mask,
                                                   unsigned long long int var,
                                                   unsigned int           delta,
                                                   int width = 32);
__device__ float __shfl_down_sync(unsigned int mask, float var, unsigned int delta, int width = 32);
__device__ double __shfl_down_sync(unsigned int mask, double var, unsigned int delta, int width = 32);

// Warp Shuffle XOR
/**
 * @brief Shuffle a variable using XOR pattern
 * @param mask Member mask for threads participating in the shuffle
 * @param var Variable to shuffle
 * @param laneMask Lane mask for XOR shuffle
 * @param width Width of the shuffle (must be power of 2, <= 32)
 * @return Value from lane (laneId XOR laneMask)
 */
__device__ int __shfl_xor_sync(unsigned int mask, int var, int laneMask, int width = 32);
__device__ unsigned int           __shfl_xor_sync(unsigned int mask,
                                                  unsigned int var,
                                                  int          laneMask,
                                                  int          width = 32);
__device__ long long int          __shfl_xor_sync(unsigned int  mask,
                                                  long long int var,
                                                  int           laneMask,
                                                  int           width = 32);
__device__ unsigned long long int __shfl_xor_sync(unsigned int           mask,
                                                  unsigned long long int var,
                                                  int laneMask,
                                                  int width = 32);
__device__ float __shfl_xor_sync(unsigned int mask, float var, int laneMask, int width = 32);
__device__ double __shfl_xor_sync(unsigned int mask, double var, int laneMask, int width = 32);

// Warp Vote Functions
/**
 * @brief Evaluate predicate for all active threads in the warp
 * @param mask Member mask for threads participating in the vote
 * @param predicate Predicate to evaluate
 * @return Non-zero if predicate is true for all active threads, zero otherwise
 */
__device__ int __all_sync(unsigned int mask, int predicate);

/**
 * @brief Evaluate predicate for any active thread in the warp
 * @param mask Member mask for threads participating in the vote
 * @param predicate Predicate to evaluate
 * @return Non-zero if predicate is true for any active thread, zero otherwise
 */
__device__ int __any_sync(unsigned int mask, int predicate);

/**
 * @brief Return a bitmask of threads that have predicate true
 * @param mask Member mask for threads participating in the ballot
 * @param predicate Predicate to evaluate
 * @return Bitmask with bit N set if predicate is true for thread N
 */
__device__ unsigned int __ballot_sync(unsigned int mask, int predicate);

// Warp Match Functions
/**
 * @brief Find threads in the warp with the same value
 * @param mask Member mask for threads participating in the match
 * @param value Value to match
 * @return Bitmask of threads with matching value
 */
__device__ unsigned int __match_any_sync(unsigned int mask, int value);
__device__ unsigned int __match_any_sync(unsigned int mask, unsigned int value);
__device__ unsigned int __match_any_sync(unsigned int mask, long long int value);
__device__ unsigned int __match_any_sync(unsigned int mask, unsigned long long int value);
__device__ unsigned int __match_any_sync(unsigned int mask, float value);

/**
 * @brief Check if all threads in the warp have the same value
 * @param mask Member mask for threads participating in the match
 * @param value Value to match
 * @return Bitmask of all threads if all match, 0 otherwise
 */
__device__ unsigned int __match_all_sync(unsigned int mask, int value);
__device__ unsigned int __match_all_sync(unsigned int mask, unsigned int value);
__device__ unsigned int __match_all_sync(unsigned int mask, long long int value);
__device__ unsigned int __match_all_sync(unsigned int mask, unsigned long long int value);
__device__ unsigned int __match_all_sync(unsigned int mask, float value);

// Warp Synchronization
/**
 * @brief Synchronize threads in the warp
 * @param mask Member mask for threads to synchronize
 */
__device__ void __syncwarp(unsigned int mask = 0xffffffff);

/**
 * @brief Return a 32-bit mask of currently active threads in the warp
 * @return Bitmask with bit N set if thread N is active
 */
__device__ unsigned int __activemask(void);

// ============================================================================
// Memory Fence Functions
// ============================================================================

/**
 * @brief Memory fence at block scope
 * 
 * Ensures that all writes to all memory made by the calling thread before
 * the call are observed by all threads in the block as occurring before all
 * writes made by the calling thread after the call. Also orders reads.
 * 
 * Equivalent to cuda::atomic_thread_fence(cuda::memory_order_seq_cst, cuda::thread_scope_block)
 */
__device__ void __threadfence_block(void);

/**
 * @brief Memory fence at device scope
 * 
 * Ensures that no writes to all memory made by the calling thread after
 * the call are observed by any thread in the device as occurring before any
 * write made by the calling thread before the call.
 * 
 * Equivalent to cuda::atomic_thread_fence(cuda::memory_order_seq_cst, cuda::thread_scope_device)
 */
__device__ void __threadfence(void);

/**
 * @brief Memory fence at system scope
 * 
 * Ensures that all writes to all memory made by the calling thread before
 * the call are observed by all threads in the device, host threads, and all
 * threads in peer devices as occurring before all writes made by the calling
 * thread after the call.
 * 
 * Equivalent to cuda::atomic_thread_fence(cuda::memory_order_seq_cst, cuda::thread_scope_system)
 * 
 * @note Only supported by devices of compute capability 2.x and higher
 */
__device__ void __threadfence_system(void);

// ============================================================================
// Synchronization Functions
// ============================================================================

/**
 * @brief Synchronize all threads in the thread block
 * 
 * Waits until all threads in the thread block have reached this point and
 * all global and shared memory accesses made by these threads prior to
 * __syncthreads() are visible to all threads in the block.
 * 
 * @note Must be called by all threads in the block, or behavior is undefined.
 *       Can be used in conditional code only if the conditional evaluates
 *       identically across the entire thread block.
 */
__device__ void __syncthreads(void);

/**
 * @brief Synchronize all threads in the thread block and count predicates
 * 
 * Identical to __syncthreads() with the additional feature that it evaluates
 * predicate for all threads of the block and returns the number of threads
 * for which predicate evaluates to non-zero.
 * 
 * @param predicate Predicate to evaluate for each thread
 * @return Number of threads for which predicate is non-zero
 * 
 * @note Only supported by devices of compute capability 2.x and higher
 */
__device__ int __syncthreads_count(int predicate);

/**
 * @brief Synchronize all threads in the thread block and compute AND of predicates
 * 
 * Identical to __syncthreads() with the additional feature that it evaluates
 * predicate for all threads of the block and returns non-zero if and only if
 * predicate evaluates to non-zero for all of them.
 * 
 * @param predicate Predicate to evaluate for each thread
 * @return Non-zero if predicate is non-zero for all threads, zero otherwise
 * 
 * @note Only supported by devices of compute capability 2.x and higher
 */
__device__ int __syncthreads_and(int predicate);

/**
 * @brief Synchronize all threads in the thread block and compute OR of predicates
 * 
 * Identical to __syncthreads() with the additional feature that it evaluates
 * predicate for all threads of the block and returns non-zero if and only if
 * predicate evaluates to non-zero for any of them.
 * 
 * @param predicate Predicate to evaluate for each thread
 * @return Non-zero if predicate is non-zero for any thread, zero otherwise
 * 
 * @note Only supported by devices of compute capability 2.x and higher
 */
__device__ int __syncthreads_or(int predicate);

// ============================================================================
// Atomic Functions
// ============================================================================

// Memory Order and Thread Scope Enums (CUDA 12.8+)
/**
 * @brief Memory order constants for atomic operations
 */
enum
{
    __NV_ATOMIC_RELAXED,
    __NV_ATOMIC_CONSUME,
    __NV_ATOMIC_ACQUIRE,
    __NV_ATOMIC_RELEASE,
    __NV_ATOMIC_ACQ_REL,
    __NV_ATOMIC_SEQ_CST
};

/**
 * @brief Thread scope constants for atomic operations
 */
enum
{
    __NV_THREAD_SCOPE_THREAD,
    __NV_THREAD_SCOPE_BLOCK,
    __NV_THREAD_SCOPE_CLUSTER,
    __NV_THREAD_SCOPE_DEVICE,
    __NV_THREAD_SCOPE_SYSTEM
};

// ============================================================================
// Arithmetic Atomic Functions
// ============================================================================

/**
 * @brief Atomic addition
 * @param address Address in global or shared memory
 * @param val Value to add
 * @return Old value at address
 */
__device__ int          atomicAdd(int* address, int val);
__device__ unsigned int atomicAdd(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicAdd(unsigned long long int* address,
                                            unsigned long long int  val);
__device__ float                  atomicAdd(float* address, float val);
__device__ double                 atomicAdd(double* address, double val);
// __device__ __half2 atomicAdd(__half2 *address, __half2 val);
// __device__ __half atomicAdd(__half *address, __half val);
// __device__ __nv_bfloat162 atomicAdd(__nv_bfloat162 *address, __nv_bfloat162 val);
// __device__ __nv_bfloat16 atomicAdd(__nv_bfloat16 *address, __nv_bfloat16 val);
// __device__ float2 atomicAdd(float2* address, float2 val);
// __device__ float4 atomicAdd(float4* address, float4 val);

/**
 * @brief Atomic subtraction
 * @param address Address in global or shared memory
 * @param val Value to subtract
 * @return Old value at address
 */
__device__ int          atomicSub(int* address, int val);
__device__ unsigned int atomicSub(unsigned int* address, unsigned int val);

/**
 * @brief Atomic exchange
 * @param address Address in global or shared memory
 * @param val Value to store
 * @return Old value at address
 */
__device__ int          atomicExch(int* address, int val);
__device__ unsigned int atomicExch(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicExch(unsigned long long int* address,
                                             unsigned long long int  val);
__device__ float                  atomicExch(float* address, float val);
// Template version for 128-bit types (compute capability 9.x+)
// template<typename T> T atomicExch(T* address, T val);

/**
 * @brief Atomic minimum
 * @param address Address in global or shared memory
 * @param val Value to compare
 * @return Old value at address
 */
__device__ int          atomicMin(int* address, int val);
__device__ unsigned int atomicMin(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicMin(unsigned long long int* address,
                                            unsigned long long int  val);
__device__ long long int atomicMin(long long int* address, long long int val);

/**
 * @brief Atomic maximum
 * @param address Address in global or shared memory
 * @param val Value to compare
 * @return Old value at address
 */
__device__ int          atomicMax(int* address, int val);
__device__ unsigned int atomicMax(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicMax(unsigned long long int* address,
                                            unsigned long long int  val);
__device__ long long int atomicMax(long long int* address, long long int val);

/**
 * @brief Atomic increment (wraps to 0 when val is reached)
 * @param address Address in global or shared memory
 * @param val Maximum value (wraps to 0 when old >= val)
 * @return Old value at address
 */
__device__ unsigned int atomicInc(unsigned int* address, unsigned int val);

/**
 * @brief Atomic decrement (wraps to val when 0 is reached)
 * @param address Address in global or shared memory
 * @param val Value to wrap to when old == 0 or old > val
 * @return Old value at address
 */
__device__ unsigned int atomicDec(unsigned int* address, unsigned int val);

/**
 * @brief Atomic compare and swap
 * @param address Address in global or shared memory
 * @param compare Value to compare with
 * @param val Value to store if compare matches
 * @return Old value at address
 */
__device__ int atomicCAS(int* address, int compare, int val);
__device__ unsigned int atomicCAS(unsigned int* address, unsigned int compare, unsigned int val);
__device__ unsigned long long int atomicCAS(unsigned long long int* address,
                                            unsigned long long int  compare,
                                            unsigned long long int  val);
__device__ unsigned short int     atomicCAS(unsigned short int* address,
                                            unsigned short int  compare,
                                            unsigned short int  val);
// Template version for 128-bit types (compute capability 9.x+)
// template<typename T> T atomicCAS(T* address, T compare, T val);

// CUDA 12.8+ Atomic Functions with Memory Order and Thread Scope
/**
 * @brief Generic atomic exchange (CUDA 12.8+)
 * @param ptr Pointer to value to exchange
 * @param val Pointer to value to store
 * @param ret Pointer to store old value
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can be any data type of size 4, 8, or 16 bytes
 * @note 16-byte types require sm_90+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_exchange(T* ptr, T* val, T *ret, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Non-generic atomic exchange (CUDA 12.8+)
 * @param ptr Pointer to value to exchange
 * @param val Value to store
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be integral type of size 4, 8, or 16 bytes
 * @note 16-byte types require sm_90+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_exchange_n(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Generic atomic compare and exchange (CUDA 12.8+)
 * @param ptr Pointer to value to compare and exchange
 * @param expected Pointer to expected value (updated if comparison fails)
 * @param desired Pointer to desired value to store
 * @param weak Weak flag (ignored, uses stronger of success/failure order)
 * @param success_order Memory order on success (must be integer literal)
 * @param failure_order Memory order on failure (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return true if comparison succeeded and value was stored, false otherwise
 * @note T can be any data type of size 2, 4, 8, or 16 bytes
 * @note 16-byte types require sm_90+
 * @note 2-byte types require sm_70+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ bool __nv_atomic_compare_exchange(T* ptr, T* expected, T* desired, bool weak, int success_order, int failure_order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Non-generic atomic compare and exchange (CUDA 12.8+)
 * @param ptr Pointer to value to compare and exchange
 * @param expected Pointer to expected value (updated if comparison fails)
 * @param desired Desired value to store
 * @param weak Weak flag (ignored, uses stronger of success/failure order)
 * @param success_order Memory order on success (must be integer literal)
 * @param failure_order Memory order on failure (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return true if comparison succeeded and value was stored, false otherwise
 * @note T can only be integral type of size 2, 4, 8, or 16 bytes
 * @note 16-byte types require sm_90+
 * @note 2-byte types require sm_70+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ bool __nv_atomic_compare_exchange_n(T* ptr, T* expected, T desired, bool weak, int success_order, int failure_order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic fetch and add (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to add
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be unsigned int, int, unsigned long long, float, or double
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_add(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic add (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to add
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be unsigned int, int, unsigned long long, float, or double
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_add(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic fetch and subtract (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to subtract
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be unsigned int, int, unsigned long long, float, or double
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_sub(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic subtract (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to subtract
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be unsigned int, int, unsigned long long, float, or double
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_sub(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic fetch and minimum (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to compare
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be unsigned int, int, unsigned long long, or long long
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_min(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic minimum (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to compare
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be unsigned int, int, unsigned long long, or long long
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_min(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic fetch and maximum (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to compare
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be unsigned int, int, unsigned long long, or long long
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_max(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic maximum (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to compare
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be unsigned int, int, unsigned long long, or long long
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_max(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

// ============================================================================
// Bitwise Atomic Functions
// ============================================================================

/**
 * @brief Atomic bitwise AND
 * @param address Address in global or shared memory
 * @param val Value to AND with
 * @return Old value at address
 */
__device__ int          atomicAnd(int* address, int val);
__device__ unsigned int atomicAnd(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicAnd(unsigned long long int* address,
                                            unsigned long long int  val);

/**
 * @brief Atomic bitwise OR
 * @param address Address in global or shared memory
 * @param val Value to OR with
 * @return Old value at address
 */
__device__ int          atomicOr(int* address, int val);
__device__ unsigned int atomicOr(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicOr(unsigned long long int* address,
                                           unsigned long long int  val);

/**
 * @brief Atomic bitwise XOR
 * @param address Address in global or shared memory
 * @param val Value to XOR with
 * @return Old value at address
 */
__device__ int          atomicXor(int* address, int val);
__device__ unsigned int atomicXor(unsigned int* address, unsigned int val);
__device__ unsigned long long int atomicXor(unsigned long long int* address,
                                            unsigned long long int  val);

/**
 * @brief Atomic fetch and OR (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to OR with
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be integral type of size 4 or 8 bytes
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_or(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic OR (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to OR with
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be integral type of size 4 or 8 bytes
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_or(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic fetch and XOR (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to XOR with
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be integral type of size 4 or 8 bytes
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_xor(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic XOR (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to XOR with
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be integral type of size 4 or 8 bytes
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_xor(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic fetch and AND (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to AND with
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Old value at ptr
 * @note T can only be integral type of size 4 or 8 bytes
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_fetch_and(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic AND (CUDA 12.8+)
 * @param ptr Pointer to value
 * @param val Value to AND with
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be integral type of size 4 or 8 bytes
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_and(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

// ============================================================================
// Other Atomic Functions
// ============================================================================

/**
 * @brief Generic atomic load (CUDA 12.8+)
 * @param ptr Pointer to value to load
 * @param ret Pointer to store loaded value
 * @param order Memory order (must be integer literal, cannot be RELEASE or ACQ_REL)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can be any data type of size 1, 2, 4, 8, or 16 bytes
 * @note 16-byte types require sm_70+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_load(T* ptr, T* ret, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Non-generic atomic load (CUDA 12.8+)
 * @param ptr Pointer to value to load
 * @param order Memory order (must be integer literal, cannot be RELEASE or ACQ_REL)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @return Loaded value
 * @note T can only be integral type of size 1, 2, 4, 8, or 16 bytes
 * @note 16-byte types require sm_70+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ T __nv_atomic_load_n(T* ptr, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Generic atomic store (CUDA 12.8+)
 * @param ptr Pointer to value to store to
 * @param val Pointer to value to store
 * @param order Memory order (must be integer literal, cannot be CONSUME, ACQUIRE, or ACQ_REL)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can be any data type of size 1, 2, 4, 8, or 16 bytes
 * @note 16-byte types require sm_70+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_store(T* ptr, T* val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Non-generic atomic store (CUDA 12.8+)
 * @param ptr Pointer to value to store to
 * @param val Value to store
 * @param order Memory order (must be integer literal, cannot be CONSUME, ACQUIRE, or ACQ_REL)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note T can only be integral type of size 1, 2, 4, 8, or 16 bytes
 * @note 16-byte types require sm_70+
 * @note Cluster scope requires sm_90+
 */
// template<typename T>
// __device__ void __nv_atomic_store_n(T* ptr, T val, int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

/**
 * @brief Atomic thread fence (CUDA 12.8+)
 * 
 * Establishes an ordering between memory accesses requested by this thread
 * based on the specified memory order. The thread scope parameter specifies
 * the set of threads that may observe the ordering effect of this operation.
 * 
 * @param order Memory order (must be integer literal)
 * @param scope Thread scope (must be integer literal, default: __NV_THREAD_SCOPE_SYSTEM)
 * @note Cluster scope requires sm_90+
 */
__device__ void __nv_atomic_thread_fence(int order, int scope = __NV_THREAD_SCOPE_SYSTEM);

#endif
