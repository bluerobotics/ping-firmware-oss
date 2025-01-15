#include "DSP/dsp.h"

#include "config.h"

uint16_t steady_state_finder(volatile uint8_t *buffer, uint16_t size, uint8_t std_deviation)
{
  uint64_t in;
  uint32_t w_acc = 0U;
  uint8_t *tail = buffer;
  uint32_t target = STEADY_STATE_WINDOW_SIZE * std_deviation;

  /** Initializes the filter window */
  uint16_t block = STEADY_STATE_WINDOW_SIZE >> 3U;
  while (block > 0)
  {
    /** Fetch 8 u8 as a u64 and update iterator 8 bytes forward */
    in = *_SIMD64(buffer)++;
    /** Add the sum of each u8 in the accumulator */
    _ACC_ADD_SIMD64(w_acc, in)

    /** Next block */
    --block;
  }

  block = (size - STEADY_STATE_WINDOW_SIZE) >> 3U;
  while (block > 0)
  {
    in = *_SIMD64(buffer)++;
    /** Next 8 samples entering filter window */
    _ACC_ADD_SIMD64(w_acc, in)
    in = *_SIMD64(tail)++;
    /** Next 8 samples leaving filter window */
    _ACC_DEC_SIMD64(w_acc, in)

    if (w_acc < target) {
      return size - (STEADY_STATE_WINDOW_SIZE << 3U);
    }

    /** Next block */
    --block;
  }
}
