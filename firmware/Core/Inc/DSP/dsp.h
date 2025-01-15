#ifndef _SONAR_DSP_H_
#define _SONAR_DSP_H_

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
#define _SIMD64(addr) (*(int64_t **) & (addr))

/**
 * @def _SIMD32
 * @brief Address reinterpretation for SIMD 32-bit operations.
 * @param addr Address to reinterpret.
 */
#define _SIMD32(addr) (*(int32_t **) & (addr))

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
#define __U32_127 0b01111111011111110111111101111111

/**
 * @brief All functions are placed in .ccmram for best performance
 */

/**
 * @brief Computes a fast mean for a buffer of uint8_t values.
 *
 * This function is optimized for buffer sizes that are integer multiples of 8.
 *
 * @param[in] buffer Pointer to the buffer to process.
 * @param[in] size Number of elements in the buffer to process.
 * @return uint8_t The computed mean value of the buffer.
 */
uint8_t u8_fast_mean(
  volatile uint8_t *buffer,
  uint16_t size
) __attribute__((section (".ccmram")));

/**
 * @brief Computes a fast standard deviation for a buffer of uint8_t values.
 *
 * This function is optimized for buffer sizes that are integer multiples of 8.
 *
 * @param[in] buffer Pointer to the buffer to process.
 * @param[in] size Number of elements in the buffer to process.
 * @param[in] mean The mean value of the buffer.
 * @return uint8_t The computed standard deviation.
 */
uint8_t u8_fast_std_dev(
  volatile uint8_t *buffer,
  uint16_t size,
  uint8_t mean
) __attribute__((section (".ccmram")));

/**
 * @brief Computes the absolute delta of each element in a buffer from a center value.
 *
 * Optimized for buffer sizes that are integer multiples of 8. For example, if a center
 * value of 127U is provided, the absolute delta is computed as:
 *
 * @code
 * value = value > 127U ? value - 127U : 127U - value;
 * @endcode
 *
 * @param[in,out] buffer Pointer to the buffer to process. The result is stored in the same buffer.
 * @param[in] size Number of elements in the buffer to process.
 * @param[in] center_pack The center value to compute the absolute delta.
 */
void u8_fast_abs_delta(
  volatile uint8_t *buffer,
  uint16_t size,
  uint32_t center_pack
) __attribute__((section (".ccmram")));

/**
 * @brief Normalizes all values in the buffer to an 8-bit maximum value (255) based on a given maximum value.
 *
 * This function is optimized for buffer sizes that are integer multiples of 8.
 *
 * The normalization formula is:
 *
 * @code
 * value = saturate(value * 256 / max);
 * @endcode
 *
 * @param[in,out] buffer Pointer to the buffer to process. The result is stored in the same buffer.
 * @param[in] size Number of elements to process in the buffer.
 * @param[in] max The maximum value used for normalization scale.
 */
void u8_fast_normalize(
  volatile uint8_t *buffer,
  uint16_t size,
  uint8_t max
) __attribute__((section (".ccmram")));

/**
 * @brief Finds the index in the buffer where the Piezo transmission resonance shadow ends.
 *
 * This function identifies the steady-state point in a signal buffer where the piezoelectric transmission's resonance
 * effect diminishes. It uses a moving window technique to evaluate when the value of the signal stabilizes bellow
 * a given standard deviation threshold.
 *
 * @note It is recommended to compute the standard deviation threshold using only the second half of the signal buffer.
 * This approach helps exclude the initial portion of the signal, which may contain transient noise or other artifacts.
 * By focusing on the later part of the signal, the computed threshold will more accurately reflect the steady-state
 * characteristics.
 *
 * @param[in] buffer Pointer to the buffer containing the signal to analyze.
 * @param[in] size Number of elements in the buffer.
 * @param[in] std_deviation Standard deviation threshold used to detect the steady state.
 *
 * @return The index in the buffer where the steady state starts (resonance shadow ends).
 *
 * @note This function is optimized for real-time embedded systems and is located in `.ccmram` for improved performance.
 */
uint16_t steady_state_finder(
  volatile uint8_t *buffer,
  uint16_t size,
  uint8_t std_deviation
) __attribute__((section (".ccmram")));

/**
 * @brief Finds the index in the buffer where the echo signal is detected.
 *
 * This function analyzes the signal buffer to locate the start of the echo signal. It uses a moving window technique
 * to evaluate when the value of the signal pass a certain threshold based on the standard deviation. And when this
 * point is located, it make a right to left search to find the first point where the signal starts
 *
 * @note It is recommended to compute the standard deviation threshold using only the second half of the signal buffer.
 * This approach helps exclude the initial portion of the signal, which may contain transient noise or other artifacts.
 * By focusing on the later part of the signal, the computed threshold will more accurately reflect the steady-state
 * characteristics.
 *
 * @param[in] buffer Pointer to the buffer containing the signal to analyze.
 * @param[in] size Number of elements in the buffer.
 * @param[in] std_deviation Standard deviation.
 *
 * @return The index in the buffer where the echo signal starts.
 *
 * @note It is recommended to carefully select the standard deviation threshold based on the expected
 * signal and noise levels to ensure accurate detection.
 */
uint16_t echo_finder(
  volatile uint8_t *buffer,
  uint16_t size,
  uint8_t std_deviation
) __attribute__((section (".ccmram")));

#ifdef __cplusplus
}
#endif

#endif /** !_SONAR_DSP_H_ */
