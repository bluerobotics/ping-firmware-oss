#include "DSP/dsp.h"

#include "config.h"

/**
 * @brief Finds the index in the buffer where the Piezo transmission resonance shadow ends.
 *
 * This function identifies the steady-state point in a signal buffer where the piezoelectric transmission's resonance
 * effect diminishes. It uses a moving window technique to evaluate when the value of the signal stabilizes bellow
 * a given standard deviation threshold.
 *
 * @note It is recommended to compute the standard deviation threshold using only the second half of the signal buffer.
 * This approach helps exclude the initial portion of the signal, which may contain transient noise or other artifacts.
 * By focusing on the later part of the signal, the computed threshold will more accurately reflect the steady-state
 * characteristics.
 *
 * @param[in] buffer Pointer to the buffer containing the signal to analyze.
 * @param[in] size Number of elements in the buffer.
 * @param[in] window_size Size of the moving window used to compute the sum of the signal.
 * @param[in] std_deviation Standard deviation threshold used to detect the steady state.
 *
 * @return The index in the buffer where the steady state starts (resonance shadow ends).
 *
 * @note This function is optimized for real-time embedded systems and is located in `.ccmram` for improved performance.
 */
uint16_t steady_state_finder(volatile uint8_t *buffer, uint16_t size, uint8_t std_deviation)
{
  uint64_t in;
  uint32_t w_acc = 0U;
  volatile uint8_t *tail = buffer;
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

  if (w_acc < target) {
    return 0U;
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
      return size - (block << 3U);
    }

    /** Next block */
    --block;
  }

  return 0U;
}
