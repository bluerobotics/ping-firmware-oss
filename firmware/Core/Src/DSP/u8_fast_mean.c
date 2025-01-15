#include "DSP/dsp.h"

/**
 * @brief Computes a fast mean for a buffer of uint8_t values.
 *
 * This function is optimized for buffer sizes that are integer multiples of 8.
 *
 * @param[in] buffer Pointer to the buffer to process.
 * @param[in] size Number of elements in the buffer to process.
 * @return uint8_t The computed mean value of the buffer.
 */
uint8_t u8_fast_mean(volatile uint8_t *buffer, uint16_t size)
{
  uint64_t in;
  uint32_t acc = 0U;

  /** We always cut unroll to 8 samples */
  uint16_t block = size >> 3U;
  while (block > 0) {
    /** Fetch 8 u8 as a u64 and update iterator 8 bytes forward */
    in = *_SIMD64(buffer)++;
    /** Add the sum of each u8 in the accumulator */
    _ACC_ADD_SIMD64(acc, in)

    /** Next block */
    --block;
  }

  return acc / size;
}
