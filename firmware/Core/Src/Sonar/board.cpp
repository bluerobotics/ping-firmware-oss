#include "adc.h"
#include "gpio.h"
#include "main.h"

#include "Sonar/board.h"

/**
 * @brief Gets the single instance of the SonarBoard class.
 * @return A reference to the static SonarBoard instance.
 */
SonarBoard &SonarBoard::GetInstance()
{
  static SonarBoard instance;
  return instance;
}

/**
 * @brief Initializes the sonar board class.
 */
void SonarBoard::init()
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_DMABufferADC1, ADC1_DMA_BUFFER_SIZE);
}

/**
 * @brief Switches the sonar board to bootloader mode for firmware updates.
 */
void SonarBoard::goToBootLoader()
{
  /** BOOT0 Stabilization */
  HAL_GPIO_WritePin(BOOT_CHARGE_PIN_GPIO_Port, BOOT_CHARGE_PIN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  /** Kill the MCU and let the IWDG do its work */
  Error_Handler();
}

/**
 * @brief Retrieves the power voltage in millivolts.
 * @return The power voltage in millivolt units.
 */
uint16_t SonarBoard::powerVoltage5MilliVolt()
{
  /** Channel 8 - Rank 1 */

  /** TODO: Linearize Voltage 5V measurement (17k78/10k) */

  return _DMABufferADC1[0];
}

/**
 * @brief Retrieves the PCB temperature in hundredths of a degree Celsius.
 * @return The PCB temperature in 0.01 °C units.
 */
uint16_t SonarBoard::PCBTemperatureCC()
{
  /** Channel 3 - Rank 2 */

  /** TODO: Add linearization of PCB Temperature Sensor */

  return _DMABufferADC1[1];
}

/**
 * @brief Retrieves the processor temperature in hundredths of a degree Celsius.
 * @return The processor temperature in 0.01 °C units.
 */
uint16_t SonarBoard::processorTemperatureCC()
{
  /** Channel Temperature Sensor - Rank 3 */

  /** TODO: Add linearization of Processor Temperature Sensor */

  return _DMABufferADC1[2];
}
