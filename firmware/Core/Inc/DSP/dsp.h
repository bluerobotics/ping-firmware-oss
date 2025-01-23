#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_gcc.h"

/**
 * @def _SIMD64
 * @brief Address reinterpretation for SIMD 64-bit operations.
 * @param addr Address to reinterpret.
 */
#define _SIMD64(addr) (*(uint64_t **) & (addr))

/**
 * @def _SIMD32
 * @brief Address reinterpretation for SIMD 32-bit operations.
 * @param addr Address to reinterpret.
 */
#define _SIMD32(addr) (*(uint32_t **) & (addr))

/**
 * @def _U32_PACK
 * @brief Pack a 8-bit value into a 32-bit integer format.
 * @param val The value to pack.
 */
#define _U32_PACK(val) ((val) | ((val) << 8U) | ((val) << 16U) | ((val) << 24U))

/**
 * @brief Shift based approach is faster for _ACC_ADD_SIMD64 and _ACC_DEC_SIMD64.
 *
 * Eg: To get first u8 (VV)
 * (in) XX XX XX XX XX XX XX VV
 * in << 56U
 * (in) VV 00 00 00 00 00 00 00
 * in >> 56U
 * (in) 00 00 00 00 00 00 00 VV
 *
 * Eg: To get second u8 (VV)
 * (in) XX XX XX XX XX XX VV XX
 * in << 48U
 * (in) VV XX 00 00 00 00 00 00
 * in >> 56U
 * (in) 00 00 00 00 00 00 00 VV
 *
 * And so on... the same logic applies to _ACC_ADD_SIMD32 and _ACC_DEC_SIMD32, only using 24 (3 bytes);
 */

/**
 * @brief Accumulate the sum of all bytes in a 64-bit integer into the accumulator.
 *
 * @param acc The accumulator variable to which the bytes will be added.
 * @param in A 64-bit integer input whose bytes will be summed and added to the accumulator.
 */
#define _ACC_ADD_SIMD64(acc, in)  \
  acc += ((in << 56U) >> 56U);    \
  acc += ((in << 48U) >> 56U);    \
  acc += ((in << 40U) >> 56U);    \
  acc += ((in << 32U) >> 56U);    \
  acc += ((in << 24U) >> 56U);    \
  acc += ((in << 16U) >> 56U);    \
  acc += ((in <<  8U) >> 56U);    \
  acc +=  (in >> 56U);

/**
 * @brief Accumulate the difference of all bytes in a 64-bit integer into the accumulator.
 *
 * @param acc The accumulator variable from which the bytes will be subtracted.
 * @param in A 64-bit integer input whose bytes will be summed and subtracted from the accumulator.
 */
#define _ACC_DEC_SIMD64(acc, in)  \
  acc -= ((in << 56U) >> 56U);    \
  acc -= ((in << 48U) >> 56U);    \
  acc -= ((in << 40U) >> 56U);    \
  acc -= ((in << 32U) >> 56U);    \
  acc -= ((in << 24U) >> 56U);    \
  acc -= ((in << 16U) >> 56U);    \
  acc -= ((in <<  8U) >> 56U);    \
  acc -=  (in >> 56U);

/**
 * @brief Accumulate the sum of all bytes in a 32-bit integer into the accumulator.
 *
 * @param acc The accumulator variable to which the bytes will be added.
 * @param in A 32-bit integer input whose bytes will be summed and added to the accumulator.
 */
#define _ACC_ADD_SIMD32(acc, in)  \
  acc += ((in << 24U) >> 24U);    \
  acc += ((in << 16U) >> 24U);    \
  acc += ((in <<  8U) >> 24U);    \
  acc +=  (in >> 24U);

/**
 * @brief Accumulate the difference of all bytes in a 32-bit integer into the accumulator.
 *
 * @param acc The accumulator variable from which the bytes will be subtracted.
 * @param in A 32-bit integer input whose bytes will be summed and subtracted from the accumulator.
 */
#define _ACC_DEC_SIMD32(acc, in)  \
  acc -= ((in << 24U) >> 24U);    \
  acc -= ((in << 16U) >> 24U);    \
  acc -= ((in <<  8U) >> 24U);    \
  acc -=  (in >> 24U);

/**
 * @brief Accumulate the sum of the squares of all bytes in a 64-bit integer into the accumulator.
 *
 * @param acc The accumulator variable to which the squares of the bytes will be added.
 * @param in A 64-bit integer input whose bytes will be squared and added to the accumulator.
 */
#define _ACC_ADD_SQ_SIMD64(acc, in)                   \
  acc += ((in << 56U) >> 56U) * ((in << 56U) >> 56U); \
  acc += ((in << 48U) >> 56U) * ((in << 48U) >> 56U); \
  acc += ((in << 40U) >> 56U) * ((in << 40U) >> 56U); \
  acc += ((in << 32U) >> 56U) * ((in << 32U) >> 56U); \
  acc += ((in << 24U) >> 56U) * ((in << 24U) >> 56U); \
  acc += ((in << 16U) >> 56U) * ((in << 16U) >> 56U); \
  acc += ((in <<  8U) >> 56U) * ((in <<  8U) >> 56U); \
  acc +=  (in >> 56U) * (in >> 56U);

/**
 * @brief Subtract the sum of the squares of all bytes in a 64-bit integer from the accumulator.
 *
 * @param acc The accumulator variable from which the squares of the bytes will be subtracted.
 * @param in A 64-bit integer input whose bytes will be squared and subtracted from the accumulator.
 */
#define _ACC_DEC_SQ_SIMD64(acc, in)                   \
  acc -= ((in << 56U) >> 56U) * ((in << 56U) >> 56U); \
  acc -= ((in << 48U) >> 56U) * ((in << 48U) >> 56U); \
  acc -= ((in << 40U) >> 56U) * ((in << 40U) >> 56U); \
  acc -= ((in << 32U) >> 56U) * ((in << 32U) >> 56U); \
  acc -= ((in << 24U) >> 56U) * ((in << 24U) >> 56U); \
  acc -= ((in << 16U) >> 56U) * ((in << 16U) >> 56U); \
  acc -= ((in <<  8U) >> 56U) * ((in <<  8U) >> 56U); \
  acc -=  (in >> 56U) * (in >> 56U);

/**
 * @brief Accumulate the sum of the squares of all bytes in a 32-bit integer into the accumulator.
 *
 * @param acc The accumulator variable to which the squares of the bytes will be added.
 * @param in A 32-bit integer input whose bytes will be squared and added to the accumulator.
 */
#define _ACC_ADD_SQ_SIMD32(acc, in)                   \
  acc += ((in << 24U) >> 24U) * ((in << 24U) >> 24U); \
  acc += ((in << 16U) >> 24U) * ((in << 16U) >> 24U); \
  acc += ((in <<  8U) >> 24U) * ((in <<  8U) >> 24U); \
  acc +=  (in >> 24U) * (in >> 24U);

/**
 * @brief Subtract the sum of the squares of all bytes in a 32-bit integer from the accumulator.
 *
 * @param acc The accumulator variable from which the squares of the bytes will be subtracted.
 * @param in A 32-bit integer input whose bytes will be squared and subtracted from the accumulator.
 */
#define _ACC_DEC_SQ_SIMD32(acc, in)                   \
  acc -= ((in << 24U) >> 24U) * ((in << 24U) >> 24U); \
  acc -= ((in << 16U) >> 24U) * ((in << 16U) >> 24U); \
  acc -= ((in <<  8U) >> 24U) * ((in <<  8U) >> 24U); \
  acc -=  (in >> 24U) * (in >> 24U);

/**
 * @def __U32_127
 * @brief Convenience macro for four 8-bit unsigned integers spread across a 32-bit value.
 *        Binary representation: 01111111 01111111 01111111 01111111.
 */
#define __U32_127 2139062143U

/**
 * @brief All functions are placed in .ccmram for best performance
 */

uint16_t steady_state_finder(
  volatile uint8_t *buffer,
  uint16_t size,
  uint16_t window_size,
  uint8_t std_deviation
) __attribute__((section (".ccmram")));

uint16_t echo_finder(
  volatile uint8_t *buffer,
  uint16_t size,
  uint16_t window_size,
  uint8_t threshold
) __attribute__((section (".ccmram")));

void u8_compress_profile(
  volatile uint8_t *input,
  uint16_t in_size,
  volatile uint8_t *output,
  uint16_t out_size
) __attribute__((section (".ccmram")));

void u8_fast_abs_delta(
  volatile uint8_t *buffer,
  uint16_t size,
  uint32_t center_pack
) __attribute__((section (".ccmram")));

uint8_t u8_fast_max(
  volatile uint8_t *buffer,
  uint16_t size
) __attribute__((section (".ccmram")));

uint8_t u8_fast_mean(
  volatile uint8_t *buffer,
  uint16_t size
) __attribute__((section (".ccmram")));

void u8_fast_normalize(
  volatile uint8_t *buffer,
  uint16_t size,
  uint8_t max
) __attribute__((section (".ccmram")));

void u8_fast_half_square(
  volatile uint8_t *buffer,
  uint16_t size
) __attribute__((section (".ccmram")));

uint8_t u8_fast_std_dev(
  volatile uint8_t *buffer,
  uint16_t size,
  uint8_t mean
) __attribute__((section (".ccmram")));

void u8_fast_threshold_cut(
  volatile uint8_t *buffer,
  uint16_t size,
  uint32_t threshold_pack
) __attribute__((section (".ccmram")));

#ifdef __cplusplus
}
#endif
