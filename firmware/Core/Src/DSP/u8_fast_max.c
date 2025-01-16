#include "DSP/dsp.h"

/**
 * @brief Finds the maximum value in an 8-bit unsigned integer buffer.
 *
 * @param[in] buffer Pointer to the buffer containing the 8-bit unsigned integers.
 * @param[in] size The number of elements in the buffer.
 *
 * @return The maximum value found in the buffer.
 */
uint8_t u8_fast_max(volatile uint8_t *buffer, uint16_t size)
{
  uint32_t max = 0U;
  /** We take 4 by 4 samples per time of the buffer */
  uint32_t *iter_buff = (uint32_t*)buffer;

  /** We always cut unroll to 8 samples */
  uint16_t block = size >> 3U;
  while (block > 0) {
    /**
     * Lest suppose:
     *  *iter_buffer = 80 174 80 174
     *  max = 200 0 0 0
     *
     * When we use USUB8 it will set GE to 1 flag for each byte of the operation if the result is greater or equal to 0
     *
     * 80 - 200 -> GE = 0
     * 174 - 0 -> GE = 1
     * 80 - 0 -> GE = 1
     * 174 - 0 -> GE = 1
     */
    __USUB8(*iter_buff, max);


    /**
     * Now we have the GE flags set for each byte of the operation, SEL will select bytes from *iter_buff and max based
     * on the GE flags, if GE is 1 it will select the byte from *iter_buff, if GE is 0 it will select the byte from max
     *
     * GE is 0 -> Select 200
     * GE is 1 -> Select 174
     * GE is 1 -> Select 80
     * GE is 1 -> Select 174
     *
     * max = 200 174 80 174
     */
    max = __SEL(*iter_buff, max);

    /** Next 4 samples */
    ++iter_buff;

    /** And so on... */

    __USUB8(*iter_buff, max);
    max = __SEL(*iter_buff, max);
    ++iter_buff;

    /** Next block */
    --block;
  }

  /** Now we have 4 individuals maximums, it's just select the global one */
  uint8_t f1 = (max >> 24U) > ((max <<  8U) >> 24U) ? (max >> 24U) : ((max <<  8U) >> 24U);
  uint8_t f2 = ((max << 16U) >> 24U) > ((max << 24U) >> 24U) ? ((max << 16U) >> 24U) : ((max << 24U) >> 24U);

  return f1 > f2 ? f1 : f2;
}
