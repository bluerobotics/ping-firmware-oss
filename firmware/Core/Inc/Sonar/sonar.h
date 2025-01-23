#pragma once

#include <cstdint>

#include "config.h"

/**
 * @enum SonarMeasurementType
 * @brief Defines the available measurement types for the sonar.
 */
enum class SonarMeasurementType : uint8_t
{
  NONE = 0,         /**< No measurement requested. */
  PROFILE,          /**< Detailed profile message containing all measurement data, including the scanned profile. */
  DISTANCE,         /**< Message with distance, confidence level, and current operational parameters. */
  DISTANCE_SIMPLE,  /**< Simplified message containing only distance and confidence level. */
};


/**
 * @enum SonarMachineState
 * @brief Represents the state of the sonar machine.
 */
enum class SonarMachineState : uint8_t
{
  IDLE = 0,           /**< System is in standby mode, no active operations. */
  TRIGGERED,          /**< Conversion has been triggered; Piezo transducer is transmitting sonar pulses. */
  SAMPLING,           /**< ADC is capturing data; DMA is transferring the samples. */
  DATA_READY,         /**< Data capture via DMA is complete; all necessary raw data is available for processing. */
  SIGNAL_PROCESSING,  /**< Digital Signal Processing (DSP) is being applied to the captured data. */
  PROFILE_GENERATED,  /**< Processed data (profile) is ready to be transmitted to the host. */
};

class PingSonar
{
public:
  static PingSonar& GetInstance();

  /** Copy and assignment should be disabled since it's a singleton */
  PingSonar(const PingSonar &other) = delete;
  void operator=(const PingSonar &) = delete;

public:
  void init();
  void update();
  void refresh();
  void asyncRefresh();
  void measure(SonarMeasurementType type);

  void oneShot();
  void shotComplete();
  void processProfile();

public:
  uint32_t transmitDuration() const { return _transmitDuration; }
  uint16_t pingNumber() const { return _pingNumber; }
  uint16_t lockedDistance() const { return _lockedDistance; }
  uint16_t dataEndIndex() const { return _sampleCycles; }
  uint8_t lockedConfidence() const { return _lockedConfidence; }

  void setRangeScanStart(uint32_t scanStart);
  uint32_t rangeScanStart() const { return _rangeScanStart; }

  uint8_t setRangeScanLength(uint32_t scanLength);
  uint32_t rangeScanLength() const { return _rangeScanLength; }

  void setSpeedOfSound(uint32_t speedOfSound);
  uint32_t speedOfSound() const { return _speedOfSound; }

  void setPingInterval(uint32_t pingInterval);
  uint32_t pingInterval() const { return _pingInterval; }

  uint8_t setNProfilePoints(uint16_t profilePoints);
  uint16_t nProfilePoints() const { return _nProfilePoints; }

  void setDeviceID(uint8_t deviceID);
  uint8_t deviceID() const { return _deviceID; }

  void setIsModeAuto(uint8_t modeAuto);
  uint8_t isModeAuto() const { return _isModeAuto; }

  void setProfileEnhanced(uint8_t profileEnhanced);
  uint8_t isProfileEnhanced() const { return _isProfileEnhanced; }

  void setProfileNormalized(uint8_t profileNormalized);
  uint8_t isProfileNormalized() const { return _isProfileNormalized; }

  uint8_t setGainSetting(uint8_t gainSetting);
  uint8_t gainSetting() const { return _gainSetting; }

  void setIsPingEnabled(uint8_t pingEnabled);
  uint8_t isPingEnabled() const { return _isPingEnabled; }

  void setIsModeContinuous(uint8_t modeContinuous);
  uint8_t isModeContinuous() const { return _isModeContinuous; }

  void setMachineState(SonarMachineState state);

protected:
  /** As it's a singleton not so worried about the init list, usig in class assign for clarity */
  PingSonar() = default;

protected:
  void adjustSonarForRange();
  void adjustADCForRange();
  void adjustTransmitForRange();
  void adjustSampleTriggerForRange();
  void adjustReceiveChainGain() const;

  uint32_t desiredSamplingFrequencyHz() const;
  uint16_t transmitRepetitionCounter() const;
  uint32_t setADCSamplingFrequencyHz(uint32_t desiredFrequencyHz) const;
  void adjustADCSamplingTime(uint32_t samplingFrequencyHz) const;
  void adjustRepetitionsForADC(uint32_t samplingFrequencyHz);

private:
  /** Server State Machine control */

  SonarMachineState _machineState = SonarMachineState::IDLE;                /**< Current sonar device state. */
  SonarMeasurementType _requestedMeasurement = SonarMeasurementType::NONE;  /**< Current in progress measurement */

  uint8_t _refreshRequired = 0U;         /**< Flag to indicate if the sonar needs to be refreshed. */
  uint32_t _lastMeasurementTick = 0U; /**< Last time a continuous measurement was triggered. */

  uint16_t _controlFlags = 0U;        /**< Control flags for the sonar device. */

  /** Sonar Operational Parameters */

  /** - Set / Get by Host */

  uint8_t _deviceID = 0U;             /**< Device ID (0-254). 255 is reserved for broadcast messages. */
  uint8_t _isModeAuto = 0U;           /**< Mode of operation: 0 for manual, 1 for automatic. */
  uint8_t _gainSetting = 6U;          /**< Current gain setting. [0-6] */
  uint8_t _isPingEnabled = 1U;        /**< Ping enable flag: 0 to disable, 1 to enable. */
  uint8_t _isModeContinuous = 1U;     /**< Continuous mode flag: 0 for single ping, 1 for continuous pings. */
  uint8_t _isProfileEnhanced = 1U;    /**< Enhanced profile flag: 0 for standard, 1 for enhanced. */
  uint8_t _isProfileNormalized = 1U;  /**< Normalized profile flag: 0 for raw, 1 for normalized. */
  uint16_t _nProfilePoints = 200U;    /**< Amount of points sent to host computer for each complete profile. */
  uint32_t _rangeScanStart = 0U;      /**< The start of the scan range. [mm] */
  uint32_t _rangeScanLength = 1000U;  /**< The length of the scan range. Minimum 1000. [mm] */
  uint32_t _pingInterval = 1000U;     /**< Interval between acoustic measurements. [ms] */
  uint32_t _speedOfSound = 342000U;   /**< The speed of sound in the measurement medium. ~1,500,000 mm/s for water. [mm/s] */

  /** - Get by Host */

  uint32_t _pingNumber = 0U;         /**< Count of pulses/measurements since boot. */
  uint32_t _lockedDistance = 0U;     /**< Distance measurement locked. [mm] */
  uint16_t _transmitDuration = 0U;   /**< Duration of the single acoustic pulse. [us] */
  uint8_t _lockedConfidence = 0U;    /**< Confidence of the distance lock. [0-100] */

  /** Internal side effects utils derived from hardware current adjust */

  float _halfSpeedOfSound = 0.0f;   /**< Half the speed of sound in the measurement medium. [mm/s] */
  float _mmPerSamplePoint = 0.0f;   /**< Length of a single sampling in the ADC buffer [mm]. */
  float _realRangeScanStart = 0.0f; /**< The real scan start length set on TIM2 [mm] */
  float _sampleInterval = 0.0f;     /**< The interval between samples [us] */
  uint16_t _sampleCycles = 0U;   /**< Total number of samples captured by the ADC */

  /** Sonar Memory Buffers */

  volatile uint8_t _DMABufferADC4[ADC4_DMA_BUFFER_SIZE]; /**< Buffer to store the ADC4 data. */
};
