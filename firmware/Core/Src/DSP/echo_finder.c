#include "DSP/dsp.h"

#include "config.h"

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
 * @param[in] window_size Size of the moving window used to compute the sum of the signal.
 * @param[in] threshold Standard deviation threshold used to detect the echo signal.
 *
 * @return The index in the buffer where the echo signal starts.
 *
 * @note It is recommended to carefully select the standard deviation threshold based on the expected
 * signal and noise levels to ensure accurate detection.
 */
uint16_t echo_finder(volatile uint8_t *buffer, uint16_t size, uint16_t window_size, uint8_t threshold)
{
  uint64_t in;
  uint32_t w_acc = 0U;
  volatile uint8_t *tail = buffer;
  /**
   * NOTE: We could probably just change for a constant but compiler is optimizing away if so right now, worths maybe
   * in future compile all DSP module as a separated library.
   */
  uint32_t target = threshold * window_size;

  /** Initializes the filter window */
  uint16_t block = window_size >> 3U;
  while (block > 0)
  {
    /** Fetch 8 u8 as a u64 and update iterator 8 bytes forward */
    in = *_SIMD64(buffer)++;
    /** Add the sum of each u8 in the accumulator */
    _ACC_ADD_SIMD64(w_acc, in)

    /** Next block */
    --block;
  }

  if (w_acc > target) {
    return 1U;
  }

  block = (size - window_size) >> 3U;
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
