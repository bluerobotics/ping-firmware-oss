#pragma once

#include <cstdint>

#include "config.h"

/**
 * @class SonarBoard
 * @brief Manages the hardware interface with the sonar device board (Only PCB parts not internal Timers etc...)
 *
 * @note It is a singleton, use GetInstance() to access the instance.
 */
class SonarBoard
{
protected:
  SonarBoard() = default;

public:
  static SonarBoard& GetInstance();

  /** Copy and assignment should be disabled since it's a singleton */
  SonarBoard(const SonarBoard &other) = delete;
  void operator=(const SonarBoard &) = delete;

public:
  void init();

  void goToBootLoader();

  uint16_t processorTemperatureCC();
  uint16_t PCBTemperatureCC();
  uint16_t powerVoltage5MilliVolt();

private:
  /** Board Memory Buffers */

  volatile uint16_t _DMABufferADC1[ADC1_DMA_BUFFER_SIZE];
};
