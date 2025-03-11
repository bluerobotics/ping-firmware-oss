#include "main.h"

#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "tim.h"

#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_opamp.h"
#include "stm32f3xx_ll_tim.h"

#include "DSP/dsp.h"
#include "Sonar/server.h"
#include "Sonar/sonar.h"

/**
 * @brief Gets the single instance of the PingSonar class.
 * @return A reference to the static PingSonar instance.
 */
PingSonar &PingSonar::GetInstance()
{
  static PingSonar instance;
  return instance;
}

/**
 * ============================
 * State Machine Control
 * ============================
 */

/**
 * @brief Initializes the Sonar Device.
 */
void PingSonar::init()
{
  /** Enable the OPAMP */
  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_OPAMP_Start(&hopamp4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the DAC for bias */
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable timers */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_OnePulse_Start(&htim1, TIM_CHANNEL_1);

  /** Refresh to inital parameters */
  refresh();
}

/**
 * @brief Update the sonar state machine.
 */
void PingSonar::update()
{
  /** Not enabled or awaiting sample */
  if (
    !_isPingEnabled || _machineState == SonarMachineState::TRIGGERED || _machineState == SonarMachineState::SAMPLING
  ) {
    return;
  }

  /** If someone requested a refresh due to parameters change */
  if (_refreshRequired != SonarRefreshType::NONE) {
    switch (_refreshRequired)
    {
      case SonarRefreshType::RANGE:
        adjustSonarForRange();
        break;
      case SonarRefreshType::GAINS:
        adjustReceiveChainGain();
        break;
      case SonarRefreshType::BOTH:
        refresh();
        break;
      default:
        break;
    }

    _refreshRequired = SonarRefreshType::NONE;
    /**
     * If the sonar is ready to process data, it is necessary to discard the current data and restart the process,
     * as the acquired data will not align with the current operational parameters
     */
    if (_machineState == SonarMachineState::DATA_READY) {
      _machineState = SonarMachineState::IDLE;
    }
  }

  /** Trigger section */

  /** TODO: Add multi sampling to avoid control being stuck when user selects slow sampling interval */
  const bool intervalExceeded = _isModeContinuous && ((HAL_GetTick() - _lastMeasurementTick) >= _pingInterval);
  if (
    _machineState == SonarMachineState::IDLE &&
    (
      _requestedMeasurement != SonarMeasurementType::NONE || intervalExceeded
    )
  ) {
    /** Starts one sampling run */
    oneShot();
  }

  /** Data processing section */

  if (_machineState == SonarMachineState::DATA_READY) {
    /** Signal Processing */
    if (_autoRangeSearching == 0U || _isModeAuto == 0U) {
      adjustSignalBias();
    }
    processProfile();
    updateEstimationsAverage();

    /** Auto adjusting */
    if (_isModeAuto) {
      processAutoRange();
    }

    ++_pingNumber;
    _machineState = SonarMachineState::PROFILE_GENERATED;
  }

  /** Data transmission section */

  if (_machineState == SonarMachineState::PROFILE_GENERATED) {
    /** Transmit Phase */
    SonarServer &server = SonarServer::GetInstance();
    server.updateMeasurement();

    /** First send required measure if needed */
    if (_requestedMeasurement != SonarMeasurementType::NONE) {
      /** Since measurements are requested by user we put in a blocking state */
      server.awaitServerTxState(SonarServerTransmissionState::IDLE, 10);
      if (server.transmitMeasurement(_requestedMeasurement) != 0) {
        return;
      }
      _requestedMeasurement = SonarMeasurementType::NONE;
    }

    /** After send profile if interval exceeded */
    if (intervalExceeded) {
      if (server.transmitMeasurement(SonarMeasurementType::PROFILE) != 0) {
        return;
      }

      _lastMeasurementTick = HAL_GetTick();
    }

    /** Relax */
    _machineState = SonarMachineState::IDLE;
  }
}

/**
 * @brief Request a refresh of the sonar device operational parameters on the next update cycle.
 */
void PingSonar::asyncRefresh(SonarRefreshType type)
{
  _refreshRequired = type;
}

/**
 * @brief Refreshes the sonar device operational parameters.
 */
void PingSonar::refresh()
{
  adjustReceiveChainGain();
  adjustSonarForRange();
}

/**
 * @brief Request sonar to start a measurement.
 *
 * @param type The type of measurement to be performed.
 */
void PingSonar::measure(SonarMeasurementType type)
{
  _requestedMeasurement = type;
}

/**
 * @brief Triggers a single shot measurement.
 */
void PingSonar::oneShot()
{
  /** We only start a new trigger sequence if no operation is being performed, otherwise we gonna override some data */
  if (_machineState != SonarMachineState::IDLE)
  {
    return;
  }
  _machineState = SonarMachineState::TRIGGERED;

  /** Prepare DMA for data collect */
  HAL_ADC_Start_DMA(&hadc4, (uint32_t *)_DMABufferADC4, _sampleCycles);

  /**
   * Trigger sequence (Uses Timer Interconnect Matrix):
   *
   * TIM8 (Piezo Transducer Drive)
   *   \- (On Enable - TRIGGERED) -> TIM2 (Scan Start Delay) [SAMPLING]
   *                                   \- (On Elapsed - SAMPLING) -> TIM1 (ADC Sampling)
   *                                                                   \- (On Elapsed - DATA_READY) -> DSP
   */
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

/**
 * @brief Indicates the sonar that data sampling is complete.
 */
void PingSonar::shotComplete()
{
  /** We check since maybe TIM1 and DMA want to change */
  if (_machineState == SonarMachineState::SAMPLING)
  {
    HAL_ADC_Stop_DMA(&hadc4);
    _machineState = SonarMachineState::DATA_READY;
  }
}

/**
 * @brief Executes the DSP sequence on the collected ADC data and generate the profile as well as estimates.
 *
 * This function runs the core of the detection and estimation, it generates a series of side effects that are used
 * for adjusting the sonar.
 *
 * @note In future we can go to a more complex envelope BPF to get better estimations mainly on stabilization indexes
 * but as this requires variety of tests in different transmit mediums we are going to keep using the search based
 * method. It takes a bit more of processing but is more reliable for unknown mediums.
 */
void PingSonar::processProfile()
{
  SonarServer &server = SonarServer::GetInstance();

  /**
   * TODO: We can in future add a next filtering step here to not only rely on hardware filters and have more control on
   * the dynamics of the signal processing.
   * Would be good to use some narrow frequency band filter to lock on the transmit frequency. Candidates could be an
   * Eliptical or Goertzel filter. Implemented as a IIR probably.
   */

  /** Start cleaning buffer and preparing for search algorithms */

  /** We aim for around 350 us for the steady state window */
  uint16_t steady_state_window_size = (uint16_t)(350.0f / _sampleInterval);
  steady_state_window_size = (steady_state_window_size >> 3U) << 3U;

  /** Center based absolute difference to convert offset AC to pulsed DC */
  u8_fast_abs_delta(_DMABufferADC4, _sampleCycles, __U32_127);

  /** Amplify differences using half square */
  if (_isProfileEnhanced) {
    u8_fast_half_square(_DMABufferADC4, _sampleCycles);
  }

  /** Usually will never be needed bu if the data does not uses the whole range scale it */
  if (_isProfileNormalized) {
    uint8_t max = u8_fast_max(_DMABufferADC4, _sampleCycles);

    if (max < 255U) {
      u8_fast_normalize(_DMABufferADC4, _sampleCycles, max);
    }
  }

  /** Extract statistical data and verifies if it is usable */
  uint16_t half_buffer_size = _sampleCycles >> 1U;
  uint8_t mean = u8_fast_mean(&_DMABufferADC4[half_buffer_size], half_buffer_size);
  uint8_t std_dev = u8_fast_std_dev(&_DMABufferADC4[half_buffer_size], half_buffer_size, mean);

  /** Clean the thrash and make it easy for the searches */
  if (_isProfileEnhanced) {
    u8_fast_threshold_cut(_DMABufferADC4, _sampleCycles, _U32_PACK(std_dev));
  }

  /** Finds where the Piezo relaxed and stopped blinding us */
  uint8_t steady_finder_threshold = (std_dev > 10U) ? std_dev : 10U;
  uint16_t steady_state_index = steady_state_finder(
    _DMABufferADC4, _sampleCycles, steady_state_window_size, steady_finder_threshold
  );
  /** We only need to search for peak after this point */
  uint16_t remainder_search_size = _sampleCycles - steady_state_index;

  /** Finds where peak is */
  uint16_t peak_index = echo_finder(
    &_DMABufferADC4[steady_state_index], remainder_search_size, ECHO_FINDER_WINDOW_SIZE, ECHO_FINDER_THRESHOLD
  );
  uint16_t final_peak_index = (peak_index + steady_state_index) >> 1U;

  if (peak_index != 0U) {
    _lockedDistance = (uint32_t)((float)(final_peak_index * _mmPerSamplePoint) + _realRangeScanStart);
    _lockedConfidence = (1.0f - ((float)std_dev / 255.0f)) * 100.0f;

    /** If standard deviation is grater than 80% or mean value is to high */
    if (std_dev > 200U || mean > 150U) {
      _lockedConfidence = 0U;
    }

    if (_lockedConfidence > 95U) {
      _lockedConfidence = 100U;
    }
  } else {
    /** No lock... */
    _lockedDistance = 0U;
    _lockedConfidence = 0U;
  }

  /** Last step is compress the profile data for user */
  u8_compress_profile(_DMABufferADC4, _sampleCycles, server.bufferProfile() + 34U, _nProfilePoints);
}

void PingSonar::updateEstimationsAverage()
{
  /** Estimate leaving element */
  uint32_t leavingConfidence = _lockedConfidenceAcc / _lockedValuesWindowSize;
  uint32_t leavingDistance = _lockedDistanceAcc / _lockedValuesWindowSize;

  /** Update accumulators */
  _lockedConfidenceAcc -= leavingConfidence;
  _lockedDistanceAcc -= leavingDistance;
  _lockedConfidenceAcc += _lockedConfidence;
  _lockedDistanceAcc += _lockedDistance;

  /** Update averages */
  _lockedConfidenceAvg = _lockedConfidenceAcc / _lockedValuesWindowSize;
  _lockedDistanceAvg = _lockedDistanceAcc / _lockedValuesWindowSize;

  if (_lockedConfidenceAvg > 95U) {
    _lockedConfidenceAvg = 100U;
  }
}

void PingSonar::resetEstimationsAverage()
{
  _lockedConfidenceAcc = 0U;
  _lockedDistanceAcc = 0U;
  _lockedConfidenceAvg = 0U;
  _lockedDistanceAvg = 0U;
}

void PingSonar::updateRangeBoundaries()
{
  uint32_t _scanLength = (uint32_t)((float)_lockedDistanceAvg / AUTO_RANGE_TARGET_CENTER_AT);
  uint32_t halfDelta = (uint32_t)((float)_scanLength * (AUTO_RANGE_BOUNDARY_DELTA / 2.0f));

  int32_t newLowerBoundary = (int32_t)_lockedDistanceAvg - (int32_t)halfDelta;
  newLowerBoundary = newLowerBoundary < 0 ? 0 : newLowerBoundary;
  _lowerRangeBoundary = (uint32_t)newLowerBoundary;

  _upperRangeBoundary = _lockedDistanceAvg + halfDelta;
  _upperRangeBoundary = _upperRangeBoundary > _scanLength ? _scanLength : _upperRangeBoundary;

  /** Clamp values */
  _scanLength = _scanLength < 1000U ? 1000U : _scanLength;

  setRangeScanLength(_scanLength);
  asyncRefresh(SonarRefreshType::RANGE);
}

/**
 * @brief Executes the auto ranging sequence based on collected profile data and estimates.
 *
 * This functions controls the effective range that the sonar should operate, it tries to keep the target at around 70%
 * of the range scan length. In case the target is lock it starts a sweep to find the best range to operate.
 */
void PingSonar::processAutoRange()
{
  /** TODO: Add auto gain estimation based on std dev */

  /** We can only adjust the range with a stable bias */
  if (_signalBiasStable == 0U) {
    _autoRangeSearching = 0U;
    return;
  }

  /** Auto ranging trigger */
  if (_lockedConfidenceAvg < AUTO_RANGE_CONFIDENCE_THRESHOLD) {
    if (!_autoRangeSearching) {
      _autoRangeSearching = 1U;
      _autoRangeLockingFor = 0U;

      resetEstimationsAverage();
      setRangeScanLength(AUTO_RANGE_SWEEP_START_MM);
      asyncRefresh(SonarRefreshType::RANGE);
      return;
    }

    if (_autoRangeLockingFor > _lockedValuesWindowSize) {
      _autoRangeLockingFor = 0U;
      resetEstimationsAverage();

      if (
        _rangeScanLength < AUTO_RANGE_SWEEP_END_MM &&
        (_rangeScanLength + AUTO_RANGE_SWEEP_STEP_MM) < _maxScanLength
      ) {
        setRangeScanLength(_rangeScanLength + AUTO_RANGE_SWEEP_STEP_MM);
        asyncRefresh(SonarRefreshType::RANGE);
      } else {
        _autoRangeSearching = 0U;
      }
    }

    _autoRangeLockingFor++;
  }

  if (_lockedConfidenceAvg >= AUTO_RANGE_CONFIDENCE_THRESHOLD) {
    /** Was searching and found the range */
    if (_autoRangeSearching) {
      updateRangeBoundaries();
      _autoRangeSearching = 0U;
    }

    if (_lockedDistanceAvg < _lowerRangeBoundary || _lockedDistanceAvg > _upperRangeBoundary) {
      updateRangeBoundaries();
    }
  }
}

/**
 * ============================
 * Hardware Adjusting
 * ============================
 */

/**
 * @brief Adjusts the sonar hardware configuration based on current set operational parameters.
 */
void PingSonar::adjustSonarForRange()
{
  /** Since we use this in a lot of place we just calculate here */
  _halfSpeedOfSound = (float)_speedOfSound / 2.0f;
  _maxScanLength = (ADC4_DMA_BUFFER_SIZE / MIN_SAMPLING_FREQUENCY_HZ) * _halfSpeedOfSound;

  /** The following functions have side effects. Be cautious when changing their order. */
  adjustTransmitForRange();
  adjustSampleTriggerForRange();
  adjustADCForRange();
}

/**
 * @brief Adjusts the receive chain gain settings based on the current gain configuration.
 *
 * @note The function assumes that `_gainSetting` is within the valid range (0 to 6).
 *       An invalid value will result in no changes to the OPAMP configuration.
 *
 * @exception Calls `Error_Handler()` if OPAMP self-calibration fails. Triggering a IWDG reset.
 */
void PingSonar::adjustReceiveChainGain()
{
  HAL_OPAMP_Stop(&hopamp2);
  HAL_OPAMP_Stop(&hopamp3);
  HAL_OPAMP_Stop(&hopamp4);

  switch (_gainSetting)
  {
    case 0:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_2);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_2);
      break;
    case 1:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_2);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_4);
      break;
    case 2:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_4);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_4);
      break;
    case 3:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_4);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_8);
      break;
    case 4:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_8);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_8);
      break;
    case 5:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_8);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_16);
      break;
    case 6:
      LL_OPAMP_SetPGAGain(hopamp2.Instance, LL_OPAMP_PGA_GAIN_16);
      LL_OPAMP_SetPGAGain(hopamp3.Instance, LL_OPAMP_PGA_GAIN_16);
      break;
    default:
      break;
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  HAL_OPAMP_Start(&hopamp4);

  /** When PGA gains are changed we need to adjust the sonar bias */
  adjustSignalBias();
}

/**
 * @brief Adjusts the receive chain bias to keep signal centered on the ideal ADC range.
 */
void PingSonar::adjustSignalBias()
{
  constexpr uint32_t MAX_DAC = 4095U;

  /** Get current bias voltage */
  uint32_t dac = HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1);

  uint16_t window_size = _sampleCycles >> 1U;
  /** We dont need to much points */
  if (window_size > DAC_BIAS_ADJUST_MEAN_WINDOW_SIZE)
  {
    window_size = DAC_BIAS_ADJUST_MEAN_WINDOW_SIZE;
  }
  /** Only the remaining from the bufferr */
  uint16_t start_index = _sampleCycles - window_size;
  uint8_t avg = u8_fast_mean(&_DMABufferADC4[start_index], window_size);

  /** We need to adjust the bias to the center proportional to the error */
  uint8_t diff = 255U;
  if (avg > DESIRED_SIGNAL_CENTER_VALUE) {
    diff = avg - DESIRED_SIGNAL_CENTER_VALUE;
    dac = (dac > diff) ? dac - diff : 0U;
  } else {
    diff = DESIRED_SIGNAL_CENTER_VALUE - avg;
    dac = (dac + diff > MAX_DAC) ? MAX_DAC : dac + diff;
  }

  _signalBiasStable = diff < 1U;

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac);
}

/**
 * @brief Adjust the ADC sampling time based on the provided sampling frequency.
 *
 * @note For frequencies higher than 2,618,180 Hz, the sampling time is set to 7 cycles,
 *       which may result in signal loss and is not recommended.
 *
 * @warning High-frequency sampling may lead to signal degradation.
 */
void PingSonar::adjustADCSamplingTime(uint32_t samplingFrequencyHz) const
{
  if (samplingFrequencyHz < 1028570) {
    LL_ADC_SetChannelSamplingTime(hadc4.Instance, LL_ADC_CHANNEL_5, ADC_SAMPLETIME_61CYCLES_5);
    return;
  }

  if (samplingFrequencyHz < 2618180) {
    LL_ADC_SetChannelSamplingTime(hadc4.Instance, LL_ADC_CHANNEL_5, ADC_SAMPLETIME_19CYCLES_5);
    return;
  }

  /** Not recommended since a lot of the signal is lost */
  LL_ADC_SetChannelSamplingTime(hadc4.Instance, LL_ADC_CHANNEL_5, ADC_SAMPLETIME_7CYCLES_5);
}

/**
 * @brief Get the desired ADC sampling frequency in hertz.
 *
 * Calculates the desired sampling function based on the range scan length to maximize data within the DMA buffer,
 * ensuring it is an integer multiple of `DRIVE_FREQUENCY_HZ` and within the range defined by
 * `MIN_SAMPLING_FREQUENCY_HZ` and `MAX_SAMPLING_FREQUENCY_HZ`.
 *
 * @return The desired sampling frequency in hertz.
 */
uint32_t PingSonar::desiredSamplingFrequencyHz() const
{
  const float mmPerPoint = (float)_rangeScanLength / (float)ADC4_DMA_BUFFER_SIZE;
  const float desiredFrequencyHz = _halfSpeedOfSound / mmPerPoint;

  /** Adjust desired frequency to be a integer multiple of DRIVE_FREQUENCY_HZ */
  const uint32_t adjustedFrequencyHz = (uint32_t)(desiredFrequencyHz / DRIVE_FREQUENCY_HZ) * DRIVE_FREQUENCY_HZ;

  /** Cuts if needed */
  if (adjustedFrequencyHz < MIN_SAMPLING_FREQUENCY_HZ)
  {
    return MIN_SAMPLING_FREQUENCY_HZ;
  }

  if (adjustedFrequencyHz > MAX_SAMPLING_FREQUENCY_HZ)
  {
    return MAX_SAMPLING_FREQUENCY_HZ;
  }

  return (uint32_t)adjustedFrequencyHz;
}

/**
 * @brief Sets the ADC sampling frequency based on the desired frequency.
 *
 * @param desiredFrequencyHz The desired sampling frequency in hertz.
 * @return Effective sampling frequency in hertz based on the Timer TOP rounding.
 */
uint32_t PingSonar::setADCSamplingFrequencyHz(uint32_t desiredFrequencyHz) const
{
  /** ADC4 is triggered by TIM1 that uses PLLCLK * 2 */
  const uint32_t pllClk2 = HAL_RCC_GetSysClockFreq() * 2U;
  const uint16_t timerTop = (uint16_t)(pllClk2 / desiredFrequencyHz);
  const uint32_t realSamplingFrequencyHz = pllClk2 / timerTop;
  __HAL_TIM_SET_AUTORELOAD(&htim1, timerTop);

  adjustADCSamplingTime(realSamplingFrequencyHz);

  return realSamplingFrequencyHz;
}

/**
 * @brief Adjusts the number of repetitions for the ADC based on the desired sampling frequency.
 *
 * @param samplingFrequencyHz The desired sampling frequency in hertz.
 */
void PingSonar::adjustRepetitionsForADC(uint32_t samplingFrequencyHz)
{
  const float mmPerEchoSample = _halfSpeedOfSound / (float)samplingFrequencyHz;
  uint16_t repetitions = (uint16_t)((float)_rangeScanLength / mmPerEchoSample);

  if (repetitions > ADC4_DMA_BUFFER_SIZE)
  {
    repetitions = ADC4_DMA_BUFFER_SIZE;
  }

  /** Make sure repetitions are integer multiple of 8 */
  repetitions = (repetitions >> 3U) << 3U;

  /** We need to remove the trash when expanding the buffer */
  if (repetitions > _sampleCycles)
  {
    memset((void*)&_DMABufferADC4[_sampleCycles], 0U, repetitions - _sampleCycles);
  }

  /** We use this as side effect to later calculate locked distance */
  _mmPerSamplePoint = 2.0f * mmPerEchoSample;
  _sampleCycles = repetitions;

  /** ADC is trigger by TIM1 by N repetitions count */
  LL_TIM_SetRepetitionCounter(htim1.Instance, repetitions - 1U);
}

/**
 * @brief Adjusts the sonar hardware configuration based on the current range scan length.
 */
void PingSonar::adjustADCForRange()
{
  uint32_t samplingFrequencyHz = desiredSamplingFrequencyHz();
  uint32_t realSamplingFrequencyHz = setADCSamplingFrequencyHz(samplingFrequencyHz);

  _sampleInterval = (1.0f / (float)realSamplingFrequencyHz) * 1e6f;

  adjustRepetitionsForADC(realSamplingFrequencyHz);
}

/**
 * @brief Adjusts the sample trigger for the scan start point.
 *
 * Configures the sample trigger for the scan start point. This adjusts TIM2 to trigger the ADC at the correct time
 * based on the medium's speed of sound and the desired start point.
 */
void PingSonar::adjustSampleTriggerForRange()
{
  /** Sample trigger is controlled by TIM2 that uses PLLCLK * 2 */
  const uint32_t pllClk2 = HAL_RCC_GetSysClockFreq() * 2U;
  float timerTop = ((float)pllClk2 * ((float)_rangeScanStart / _halfSpeedOfSound));
  timerTop = timerTop / 2.0f;

  uint32_t top = (uint32_t)timerTop;

  if (top < 1U)
  {
    top = 1U;
  }

  _realRangeScanStart = ((float)top * _halfSpeedOfSound) / (float)pllClk2;

  __HAL_TIM_SET_AUTORELOAD(&htim2, top);
}

/**
 * @brief Calculates the number of repetitions for the Piezo transmit based on the desired resolution.
 *
 * @return The number of repetitions for the Piezo transmit.
 */
uint16_t PingSonar::transmitRepetitionCounter() const
{
  const float singleExcitationLength = (1.0f / (float)DRIVE_FREQUENCY_HZ) * _speedOfSound;

  /**
   * NOTE: This one may seems like a bug, and for sure is not "Ideally" correct, since the range resolution is should
   * only be based on scan length, but since this determines amount of power transmitted, we need to take in account
   * the scan start as well to avoid cases where range is small but start is big.
   */
  const float scanRange = (float)(_rangeScanStart + _rangeScanLength);
  const float desiredResolution = DESIRED_RANGE_RESOLUTION * scanRange;

  const uint16_t repetitions = (uint16_t)(desiredResolution / singleExcitationLength);

  /** We allow minimum of 4 repetitions, otherwise almost no power will be transmitted on the water */
  return repetitions < MIN_TRANSMIT_REPETITIONS ? MIN_TRANSMIT_REPETITIONS : repetitions;
}

/**
 * @brief Adjusts the transmit duration based on the scan range.
 */
void PingSonar::adjustTransmitForRange()
{
  /** Adjust transmit interval of the Piezo based on scan range */
  uint16_t reps = transmitRepetitionCounter();

  /** Piezo is drive by TIM8 by N repetitions count */
  LL_TIM_SetRepetitionCounter(htim8.Instance, reps);

  /** We use this as side effect to later supply transmit duration [us] for host */
  const float singleExcitationPeriod = (1.0f / (float)DRIVE_FREQUENCY_HZ);
  _transmitDuration = (uint16_t)((float)reps * singleExcitationPeriod * 1e6f);
}

/**
 * Setters
 */

/**
 * @brief Set the _deviceID
 *
 * @param deviceID The value to set.
 */
void PingSonar::setDeviceID(uint8_t deviceID)
{
  _deviceID = deviceID;
}

/**
 * @brief Set the _gainSetting
 *
 * @param gainSetting The value to set.
 * @return HAL_OK if the value is within the valid range, otherwise HAL_ERROR.
 */
uint8_t PingSonar::setGainSetting(uint8_t gainSetting)
{
  /** Must be between 0 and 6 */
  if (gainSetting > 6)
  {
    return HAL_ERROR;
  }

  _gainSetting = gainSetting;
  asyncRefresh(SonarRefreshType::GAINS);

  return HAL_OK;
}

/**
 * @brief Set the _isPingEnabled
 *
 * @param pingEnabled The value to set.
 */
void PingSonar::setIsPingEnabled(uint8_t pingEnabled)
{
  _isPingEnabled = pingEnabled;
}

/**
 * @brief Set the _isModeAuto
 *
 * @param modeAuto The value to set.
 */
void PingSonar::setIsModeAuto(uint8_t modeAuto)
{
  _isModeAuto = modeAuto;
}

/**
 * @brief Set the _isProfileEnhanced
 *
 * @param profileEnhanced The value to set.
 */
void PingSonar::setProfileEnhanced(uint8_t profileEnhanced)
{
  _isProfileEnhanced = profileEnhanced;
}

/**
 * @brief Set the _isProfileNormalized
 *
 * @param profileNormalized The value to set.
 */
void PingSonar::setProfileNormalized(uint8_t profileNormalized)
{
  _isProfileNormalized = profileNormalized;
}

/**
 * @brief Set the _isModeContinuous
 *
 * @param modeContinuous The value to set.
 */
void PingSonar::setIsModeContinuous(uint8_t modeContinuous)
{
  _isModeContinuous = modeContinuous;
}

/**
 * @brief Set the _nProfilePoints
 *
 * @param profilePoints The value to set.
 * @return HAL_OK if the value is within the valid range, otherwise HAL_ERROR.
 */
uint8_t PingSonar::setNProfilePoints(uint16_t profilePoints)
{
  /** Must be between 1 and ADC4_DMA_BUFFER_SIZE */
  if (profilePoints < 1 || profilePoints > PROFILE_MSG_BUFFER_SIZE)
  {
    return HAL_ERROR;
  }

  _nProfilePoints = profilePoints;

  return HAL_OK;
}

/**
 * @brief Set the _rangeScanStart
 *
 * @param scanStart The value to set.
 */
void PingSonar::setRangeScanStart(uint32_t scanStart)
{
  _rangeScanStart = scanStart;
}

/**
 * @brief Set the _rangeScanLength
 *
 * @param scanLength The value to set.
 * @return HAL_OK if the value is greater than 1000, otherwise HAL_ERROR.
 */
uint8_t PingSonar::setRangeScanLength(uint32_t scanLength)
{
  /** Clamp values */
  if (scanLength < 1000U || scanLength > _maxScanLength)
  {
    return HAL_ERROR;
  }

  _rangeScanLength = scanLength;
  asyncRefresh(SonarRefreshType::RANGE);

  return HAL_OK;
}

/**
 * @brief Set the _speedOfSound
 *
 * @param speedOfSound The value to set.
 */
void PingSonar::setSpeedOfSound(uint32_t speedOfSound)
{
  _speedOfSound = speedOfSound;
  _halfSpeedOfSound = (float)speedOfSound / 2.0f;
  _maxScanLength = (ADC4_DMA_BUFFER_SIZE / MIN_SAMPLING_FREQUENCY_HZ) * _halfSpeedOfSound;

  /** Clamp the scan length to maximum allowed */
  if (_rangeScanLength > _maxScanLength)
  {
    _rangeScanLength = _maxScanLength;
  }

  asyncRefresh(SonarRefreshType::RANGE);
}

/**
 * @brief Set the _pingInterval
 *
 * @param pingInterval The value to set.
 */
void PingSonar::setPingInterval(uint32_t pingInterval)
{
  _pingInterval = pingInterval;
}

/**
 * @brief Set the _pingNumber
 *
 * @param pingNumber The value to set.
 */
void PingSonar::setMachineState(SonarMachineState state)
{
  _machineState = state;
}

/** Interruptions */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  PingSonar &sonar = PingSonar::GetInstance();

  /** We started sampling */
  if (htim == &htim2)
  {
    sonar.setMachineState(SonarMachineState::SAMPLING);
  }

  /** This one is usually faster than DMA interrupt so we use both but shouldn't be needed DMA one */
  if (htim == &htim1)
  {
    sonar.shotComplete();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /** We only care about the ADC4 others don't matter here */
  if (hadc != &hadc4)
  {
    return;
  }

  PingSonar &sonar = PingSonar::GetInstance();

  /** Fallback in case TIM2 didn't changed the state */
  sonar.shotComplete();
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  /** We only care about the ADC4 others don't matter here */
  if (hadc != &hadc4)
  {
    return;
  }

  /** Should not happen, but if so :/ */
  PingSonar &sonar = PingSonar::GetInstance();
  sonar.setMachineState(SonarMachineState::IDLE);
}
