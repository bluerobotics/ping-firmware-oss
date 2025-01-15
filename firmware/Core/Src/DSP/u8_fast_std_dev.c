#include "DSP/dsp.h"

#include <math.h>

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
uint8_t u8_fast_std_dev(volatile uint8_t *buffer, uint16_t size, uint8_t mean)
{
  uint64_t in;
  uint32_t mean_of_squares;
  uint32_t sum_of_squares = 0U;
  uint16_t square_of_mean = mean * mean;

  /** We always cut unroll to 8 samples */
  uint16_t block = size >> 3U;
  while (block > 0) {
    /** Fetch 8 u8 as a u64 and update iterator 8 bytes forward */
    in = *_SIMD64(buffer)++;
    /** Add the sum of squares of each u8 in the accumulator */
    _ACC_DEC_SQ_SIMD64(sum_of_squares, in)

    /** Next block */
    --block;
  }

  /**
   * Computes the mean of squares
   *
   * The effective loop size can be calculated by dividing the size by 8 truncating the remainder
   * and then multiplying by 8 to get the effective size.
   *
   * Eg: size = 26 (3 * 8 + 2)
   * (size >> 3U) = 3
   * (1 << 3U) = 24
   * So the effective size is 24
   */
  mean_of_squares = sum_of_squares / ((size >> 3U) << 3U);

  /** Computes std deviation */
  return sqrt(mean_of_squares - square_of_mean);
}
