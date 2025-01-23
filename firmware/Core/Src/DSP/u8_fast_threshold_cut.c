#include "DSP/dsp.h"

/**
 * @brief Cut the buffer based on a threshold value pack.
 *
 * Optimized for buffer sizes that are integer multiples of 8. For example, if a threshold
 * value of 127U is provided, all values bellow this will be zero and all values above will
 * remain the same. The operation is computed as:
 *
 * @code
 * value = value < threshold ? 0U : value;
 * @endcode
 *
 * @param[in,out] buffer Pointer to the buffer to process. The result is stored in the same buffer.
 * @param[in] size Number of elements in the buffer to process.
 * @param[in] threshold_pack The threshold value to compute the absolute delta.
 */
void u8_fast_threshold_cut(volatile uint8_t *buffer, uint16_t size, uint32_t threshold_pack)
{
  /** We take 4 by 4 samples per time of the buffer */
  uint32_t *iter_buff = (uint32_t*)buffer;

  uint16_t block = size >> 3U;
  while (block > 0) {
    /** This one is trick, but it uses the GE flag on the CPU to simulate 4 ifs at once and the operation */

    /**
     * Lest suppose *iter_buffer = 5 50 8 20 and threshold_pack = 10 10 10 10
     *
     * When we use USUB8 it will set GE to 1 flag for each byte of the operation if the result is greater or equal to 0
     *
     * 5 - 10 -> GE = 0
     * 50 - 10 -> GE = 1
     * 8 - 10 -> GE = 0
     * 20 - 10 -> GE = 1
     */
    __USUB8(*iter_buff, threshold_pack);

    /**
     * Now we have the GE flags set for each byte of the operation, SEL will select bytes from *iter_buff and 0 based
     * on the GE flags, if GE is 1 it will select the byte from *iter_buff, if GE is 0 it will select the byte from
     * 0
     *
     * GE is 0 -> Select 0
     * GE is 1 -> Select 50
     * GE is 0 -> Select 0
     * GE is 1 -> Select 20
     *
     * iter_buff = 0 50 0 20
     */
    *iter_buff = __SEL(*iter_buff, 0U);

    /** Next 4 samples */
    ++iter_buff;

    /** And so on... */

    __USUB8(*iter_buff, threshold_pack);
    *iter_buff = __SEL(*iter_buff, 0U);
    ++iter_buff;

    /** Next block */
    --block;
  }
}
