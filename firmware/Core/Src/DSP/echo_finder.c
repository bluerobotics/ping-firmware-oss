#include "DSP/dsp.h"

#include "config.h"

uint16_t echo_finder(volatile uint8_t *buffer, uint16_t size, uint8_t std_deviation)
{
  uint64_t in;
  uint32_t w_acc = 0U;
  volatile uint8_t *tail = buffer;
  /** TODO: Change this 4U based on how big std_dev is compared to 255U */
  uint32_t target = ECHO_WINDOW_SIZE * std_deviation * 4U;

  /** Initializes the filter window */
  uint16_t block = ECHO_WINDOW_SIZE >> 3U;
  while (block > 0)
  {
    /** Fetch 8 u8 as a u64 and update iterator 8 bytes forward */
    in = *_SIMD64(buffer)++;
    /** Add the sum of each u8 in the accumulator */
    _ACC_ADD_SIMD64(w_acc, in)

    /** Next block */
    --block;
  }

  block = (size - ECHO_WINDOW_SIZE) >> 3U;
  while (block > 0)
  {
    in = *_SIMD64(buffer)++;
    /** Next 8 samples entering filter window */
    _ACC_ADD_SIMD64(w_acc, in)
    in = *_SIMD64(tail)++;
    /** Next 8 samples leaving filter window */
    _ACC_DEC_SIMD64(w_acc, in)

    if (w_acc > target) {
      return size - (block << 3U);
    }

    /** Next block */
    --block;
  }

  /** TODO: Add left to right descent */
  return 0U;
}
