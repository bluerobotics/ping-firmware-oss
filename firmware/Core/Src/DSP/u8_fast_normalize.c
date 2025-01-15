#include "DSP/dsp.h"

/**
 * @brief Normalizes all values in the buffer to an 8-bit maximum value (255) based on a given maximum value.
 *
 * This function is optimized for buffer sizes that are integer multiples of 8. Be aware that unlike other functions,
 * this function does not optimize so well the processing, only being able to process 2 samples per iteration. So it
 * is recommended to take in account that for large buffers, the performance can degrade.
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
void u8_fast_normalize(volatile uint8_t *buffer, uint16_t size, uint8_t max)
{
  /** Operation register and saturation result */
  uint32_t reg;
  uint32_t sat;
  /** Iterators for convenient access */
  uint16_t *iter_reg = (uint16_t*)&reg;
  uint8_t *iter_sat = (uint8_t*)&sat;
  /** Computes the scaling factor based on a u16 for best precision */
  uint16_t factor = (1 << 16) / max;

  /** We always cut unroll to 8 samples */
  uint16_t block = size >> 3U;
  while (block > 0) {
    /** Samples 0 and 1 */

    /** Store first scaled value in half of the u32 reg */
    iter_reg[0] = (buffer[0] * factor) >> 8;
    /** Store second scaled value in the other half of the u32 reg */
    iter_reg[1] = (buffer[1] * factor) >> 8;
    /** Saturate the u32 reg to u8 limits and store result in sat */
    sat = __USAT16(reg, 8U);
    /** Store the first saturated u8 scaled value back in buffer, index 1 will be always 0 */
    buffer[0] = iter_sat[0];
    /** Store the second saturated u8 scaled value back in buffer, index 3 will be always 0 */
    buffer[1] = iter_sat[2];
    /** Next 2 samples */
    ++buffer;
    ++buffer;

    /** And so on... */

    /** Samples 2 and 3 */
    iter_reg[0] = (buffer[0] * factor) >> 8;
    iter_reg[1] = (buffer[1] * factor) >> 8;
    sat = __USAT16(reg, 8U);
    buffer[0] = iter_sat[0];
    buffer[1] = iter_sat[2];
    ++buffer;
    ++buffer;

    /** Samples 4 and 5 */
    iter_reg[0] = (buffer[0] * factor) >> 8;
    iter_reg[1] = (buffer[1] * factor) >> 8;
    sat = __USAT16(reg, 8U);
    buffer[0] = iter_sat[0];
    buffer[1] = iter_sat[2];
    ++buffer;
    ++buffer;

    /** Samples 6 and 7 */
    iter_reg[0] = (buffer[0] * factor) >> 8;
    iter_reg[1] = (buffer[1] * factor) >> 8;
    sat = __USAT16(reg, 8U);
    buffer[0] = iter_sat[0];
    buffer[1] = iter_sat[2];
    ++buffer;
    ++buffer;

    /** Next block */
    --block;
  }
}
