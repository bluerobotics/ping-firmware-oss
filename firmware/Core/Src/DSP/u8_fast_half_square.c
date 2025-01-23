#include "DSP/dsp.h"

/** TODO: Need docs */
void u8_fast_half_square(volatile uint8_t *buffer, uint16_t size)
{
  /** Operation register and saturation result */
  uint32_t reg;
  uint32_t sat;
  /** Iterators for convenient access */
  uint16_t *iter_reg = (uint16_t*)&reg;
  uint8_t *iter_sat = (uint8_t*)&sat;

  /** We always cut unroll to 8 samples */
  uint16_t block = size >> 3U;
  while (block > 0) {
    /** Samples 0 and 1 */

    /** Store first square value in half of the u32 reg */
    iter_reg[0] = buffer[0] * (buffer[0] >> 1);
    /** Store second square value in the other half of the u32 reg */
    iter_reg[1] = buffer[1] * (buffer[1] >> 1);
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
    iter_reg[0] = buffer[0] * (buffer[0] >> 1);
    iter_reg[1] = buffer[1] * (buffer[1] >> 1);
    sat = __USAT16(reg, 8U);
    buffer[0] = iter_sat[0];
    buffer[1] = iter_sat[2];
    ++buffer;
    ++buffer;

    /** Samples 4 and 5 */
    iter_reg[0] = buffer[0] * (buffer[0] >> 1);
    iter_reg[1] = buffer[1] * (buffer[1] >> 1);
    sat = __USAT16(reg, 8U);
    buffer[0] = iter_sat[0];
    buffer[1] = iter_sat[2];
    ++buffer;
    ++buffer;

    /** Samples 6 and 7 */
    iter_reg[0] = buffer[0] * (buffer[0] >> 1);
    iter_reg[1] = buffer[1] * (buffer[1] >> 1);
    sat = __USAT16(reg, 8U);
    buffer[0] = iter_sat[0];
    buffer[1] = iter_sat[2];
    ++buffer;
    ++buffer;

    /** Next block */
    --block;
  }
}
