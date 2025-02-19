#include "DSP/dsp.h"

/**
 * @brief Compresses an 8-bit unsigned integer buffer by selecting the maximum value in each segment.
 *
 * This function reduces the size of the input buffer by dividing it into `out_size` segments,
 * each containing approximately `in_size / out_size` elements. The maximum value from each
 * segment is stored in the output buffer.
 *
 * @param[in] input Pointer to the input buffer containing 8-bit unsigned integers.
 * @param[in] in_size The number of elements in the input buffer.
 * @param[out] output Pointer to the output buffer where the compressed values will be stored.
 * @param[in] out_size The desired number of elements in the output buffer. If `out_size` is
 * greater than `in_size`, it will be clamped to `in_size`.
 *
 */
void u8_compress_profile(volatile uint8_t *input, uint16_t in_size, volatile uint8_t *output, uint16_t out_size)
{
  /** TODO: Need optimize this */
  if (out_size > in_size) {
    out_size = in_size;
  }

  uint16_t points_per_sector = in_size / out_size;
  uint16_t remaining_points = in_size % out_size;

  uint16_t input_index = 0;
  for (int i = 0; i < out_size; ++i)
  {
    int max_val = input[input_index];
    int sector_in_size = points_per_sector + (i == out_size - 1 ? remaining_points : 0);

    for (int j = 0; j < sector_in_size; ++j)
    {
      if (input[input_index] > max_val)
      {
        max_val = input[input_index];
      }
      ++input_index;
    }

    output[i] = max_val;
  }
}
