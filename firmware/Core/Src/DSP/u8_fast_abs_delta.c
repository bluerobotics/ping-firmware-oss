#include "DSP/dsp.h"

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
void u8_fast_abs_delta(volatile uint8_t *buffer, uint16_t size, uint32_t center_pack)
{
  /** Temporary memory for operations */
  uint32_t frac1;
  uint32_t frac2;
  /** We take 4 by 4 samples per time of the buffer */
  uint32_t *iter_buff = (uint32_t*)buffer;

  uint16_t block = size >> 3U;
  while (block > 0) {
    /** This one is trick, but it uses the GE flag on the CPU to simulate 4 ifs at once and the operation */

    /**
     * Lest suppose *iter_buffer = 80 174 80 174
     *
     * When we use USUB8 it will set GE to 1 flag for each byte of the operation if the result is greater or equal to 0
     *
     * 80 - 127 -> GE = 0
     * 174 - 127 -> GE = 1
     * 80 - 127 -> GE = 0
     * 174 - 127 -> GE = 1
     */
    __USUB8(*iter_buff, __U32_127);

    /**
     * Now we have the GE flags set for each byte of the operation, SEL will select bytes from *iter_buff and __U32_127
     * based on the GE flags, if GE is 1 it will select the byte from *iter_buff, if GE is 0 it will select the byte
     * from __U32_127
     *
     * GE is 0 -> Select 127
     * GE is 1 -> Select 174
     * GE is 0 -> Select 127
     * GE is 1 -> Select 174
     *
     * frac1 = 127 174 127 174
     */
    frac1 = __SEL(*iter_buff, __U32_127);

    /**
     * GE is 0 -> Select 80
     * GE is 1 -> Select 127
     * GE is 0 -> Select 80
     * GE is 1 -> Select 127
     *
     * frac2 = 80 127 80 127
     */
    frac2 = __SEL(__U32_127, *iter_buff);

    /**
     * Now we have the two fractions, we need to subtract 127 from the first one and the second one from 127
     *
     * 127 - 127 = 0
     * 174 - 127 = 47
     * 127 - 127 = 0
     * 174 - 127 = 47
     *
     * frac1 = 0 47 0 47
     */
    frac1 = __USUB8(frac1, __U32_127);

    /**
     * 127 - 80 = 47
     * 127 - 127 = 0
     * 127 - 80 = 47
     * 127 - 127 = 0
     *
     * frac2 = 47 0 47 0
     */
    frac2 = __USUB8(__U32_127, frac2);

    /**
     * Now we have the two fractions, we need to OR them to get the final result
     *
     * 0 | 47 = 47
     * 47 | 0 = 47
     * 0 | 47 = 47
     * 47 | 0 = 47
     *
     * *iter_buff = 47 47 47 47
     */
    *iter_buff = frac1 | frac2;

    /** Next 4 samples */
    ++iter_buff;

    /** And so on... */

    __USUB8(*iter_buff, __U32_127);
    frac1 = __SEL(*iter_buff, __U32_127);
    frac2 = __SEL(__U32_127, *iter_buff);
    frac1 = __USUB8(frac1, __U32_127);
    frac2 = __USUB8(__U32_127, frac2);
    *iter_buff = frac1 | frac2;
    ++iter_buff;

    /** Next block */
    --block;
  }
}
