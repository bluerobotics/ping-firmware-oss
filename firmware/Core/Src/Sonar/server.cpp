#include <cstdint>

#include "stm32f3xx_ll_usart.h"
#include "usart.h"

#include "config.h"
#include "Sonar/board.h"
#include "Sonar/server.h"

/**
 * @brief Gets the single instance of the SonarServer class.
 * @return A reference to the static SonarServer instance.
 */
SonarServer &SonarServer::GetInstance()
{
  static SonarServer instance;
  return instance;
}

/**
 * ============================
 * Static globals
 * ============================
 */

PingParser pingParser(2U * SINGLE_MESSAGE_SIZE);

/**
 * ============================
 * State Machine Control
 * ============================
 */

/**
 * @brief Initializes the Sonar server.
 */
void SonarServer::init()
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, static_cast<uint8_t*>(_UARTBufferRx), UART_RX_BUFFER_SIZE);
  setRxParsed(0U);
  setRxReceived(0U);
}

/**
 * @brief Executes the RX part of the server state machine.
 */
void SonarServer::processRxBuffer()
{

  /** Try to parser previous message that wasn't able to be transmitted */
  if (_pendingTxSize != 0) {
    _pendingTxSize = router(pingParser.rxMessage);
  }

  /** Consume till both are symmetric or tx is full */
  while (_UARTRxBufferParsed != _UARTRxBufferReceived && _pendingTxSize == 0)
  {
    PingParser::State parse_result = pingParser.parseByte(_UARTBufferRx[_UARTRxBufferParsed]);

    _UARTRxBufferParsed = (_UARTRxBufferParsed + 1) % UART_RX_BUFFER_SIZE;

    if (parse_result == PingParser::State::NEW_MESSAGE) {
      _pendingTxSize = router(pingParser.rxMessage);
    }
  }
}

/**
 * @brief Executes the TX part of the server state machine.
 */
void SonarServer::processTxBuffer()
{
  if (
   _serverTxState == SonarServerTransmissionState::IDLE &&
   _UARTTxBufferAvailable != UART_TX_BUFFER_SIZE
  ) {
    _serverTxState = SonarServerTransmissionState::SERVER_TX_STARTED;
    if (
      HAL_UART_Transmit_DMA(
        &huart1,
        static_cast<uint8_t*>(_UARTBufferTx),
        UART_TX_BUFFER_SIZE - _UARTTxBufferAvailable
      ) != HAL_OK
    ) {
      _serverTxState = SonarServerTransmissionState::IDLE;
      return;
    }

    /** Reset Head and cut write operations */
    _UARTTxBufferHead = _UARTBufferTx;
    _UARTTxBufferAvailable = 0;
  }
}

/**
 * @brief Update the server state machine.
 */
void SonarServer::update()
{
  processRxBuffer();
  processTxBuffer();

  /** Maintain more iterations in case of msg burst avoiding losing packages in RX */
  uint16_t lock = 1000;
  while (_pendingTxSize != 0 && lock > 0) {
    processRxBuffer();
    processTxBuffer();

    --lock;
  }
}

/**
 * @brief Waits until the server reaches a specified transmission state.
 *
 * @note This is a blocking function, use with caution.
 *
 * @param state The transmission state to wait for.
 * @param retries Retries to wait for the state each counts 1ms approximately.
 */
void SonarServer::awaitServerTxState(SonarServerTransmissionState state, uint16_t ms)
{
  while (_serverTxState != state && ms > 0) {
    --ms;
    HAL_Delay(1);
  }
}

/**
 * ============================
 * Receivers
 * ============================
 */

/**
 * @brief Run the server router for a given message.
 *
 * @param msg The message to route.
 * @return Zero if message was successfully routed, otherwise the amount of bytes pending to be written.
 */
uint8_t SonarServer::router(ping_message &msg)
{
  PingSonar &sonar = PingSonar::GetInstance();
  SonarBoard &board = SonarBoard::GetInstance();

  switch (static_cast<uint16_t>(msg.message_id())) {
    case CommonId::GENERAL_REQUEST: {
      common_general_request request = reinterpret_cast<common_general_request &>(msg);
      uint16_t requested_id = request.requested_id();
      msg.set_message_id(requested_id);

      /** Next message should not expect a payload */
      return router(msg);
    }
    case CommonId::PROTOCOL_VERSION: {
      common_protocol_version response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_version_minor(PROTOCOL_VERSION_MINOR);
      response.set_version_patch(PROTOCOL_VERSION_PATCH);
      response.updateChecksum();
      return 0;
    }
    case CommonId::DEVICE_INFORMATION: {
      common_device_information response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_device_type(PING_DEVICE_TYPE);
      response.set_device_revision(PING_DEVICE_REVISION);
      response.set_firmware_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_firmware_version_minor(PROTOCOL_VERSION_MINOR);
      response.set_firmware_version_patch(PROTOCOL_VERSION_PATCH);
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::FIRMWARE_VERSION: {
      ping1d_firmware_version response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_device_type(PING_DEVICE_TYPE);
      response.set_device_model(PING_DEVICE_MODEL);
      response.set_firmware_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_firmware_version_minor(PROTOCOL_VERSION_MINOR);
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::GENERAL_INFO: {
      ping1d_general_info response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_firmware_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_firmware_version_minor(PROTOCOL_VERSION_MINOR);
      response.set_voltage_5(board.powerVoltage5MilliVolt());
      response.set_ping_interval(sonar.pingInterval());
      response.set_gain_setting(sonar.gainSetting());
      response.set_mode_auto(sonar.isModeAuto());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_DEVICE_ID: {
      ping1d_set_device_id request = reinterpret_cast<ping1d_set_device_id &>(msg);
      sonar.setDeviceID(request.device_id());
      return 0;
    }
    case Ping1dId::DEVICE_ID: {
      ping1d_device_id response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_device_id(sonar.deviceID());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_RANGE: {
      ping1d_set_range request = reinterpret_cast<ping1d_set_range &>(msg);

      sonar.setRangeScanStart(request.scan_start());
      /** TODO: Add NACK in case of length validation fail */
      sonar.setRangeScanLength(request.scan_length());
      sonar.asyncRefresh();
      return 0;
    }
    case Ping1dId::RANGE: {
      ping1d_range response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_scan_start(sonar.rangeScanStart());
      response.set_scan_length(sonar.rangeScanLength());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_SPEED_OF_SOUND: {
      ping1d_set_speed_of_sound request = reinterpret_cast<ping1d_set_speed_of_sound &>(msg);
      sonar.setSpeedOfSound(request.speed_of_sound());
      sonar.asyncRefresh();
      return 0;
    }
    case Ping1dId::SPEED_OF_SOUND: {
      ping1d_speed_of_sound response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_speed_of_sound(sonar.speedOfSound());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_MODE_AUTO: {
      ping1d_set_mode_auto request = reinterpret_cast<ping1d_set_mode_auto &>(msg);
      sonar.setIsModeAuto(request.mode_auto());
      sonar.asyncRefresh();
      return 0;
    }
    case Ping1dId::MODE_AUTO: {
      ping1d_mode_auto response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_mode_auto(sonar.isModeAuto());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_PING_INTERVAL: {
      ping1d_set_ping_interval request = reinterpret_cast<ping1d_set_ping_interval &>(msg);
      sonar.setPingInterval(request.ping_interval());
      break;
    }
    case Ping1dId::PING_INTERVAL: {
      ping1d_ping_interval response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_ping_interval(sonar.pingInterval());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_GAIN_SETTING: {
      ping1d_set_gain_setting request = reinterpret_cast<ping1d_set_gain_setting &>(msg);
      /** TODO: Add NACK in case of gain validation fail */
      sonar.setGainSetting(request.gain_setting());
      sonar.asyncRefresh();
      return 0;
    }
    case Ping1dId::GAIN_SETTING: {
      ping1d_gain_setting response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_gain_setting(sonar.gainSetting());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::SET_PING_ENABLE: {
      ping1d_set_ping_enable request = reinterpret_cast<ping1d_set_ping_enable &>(msg);
      sonar.setIsPingEnabled(request.ping_enabled());
      return 0;
    }
    case Ping1dId::PING_ENABLE: {
      ping1d_ping_enable response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_ping_enabled(sonar.isPingEnabled());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::DISTANCE: {
      sonar.measure(SonarMeasurementType::DISTANCE);
      return 0;
    }
    case Ping1dId::DISTANCE_SIMPLE: {
      sonar.measure(SonarMeasurementType::DISTANCE_SIMPLE);
      return 0;
    }
    case Ping1dId::PROFILE: {
      sonar.measure(SonarMeasurementType::PROFILE);
      return 0;
    }
    case Ping1dId::TRANSMIT_DURATION: {
      ping1d_transmit_duration response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_transmit_duration(sonar.transmitDuration());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::PROCESSOR_TEMPERATURE: {
      ping1d_processor_temperature response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_processor_temperature(board.processorTemperatureCC());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::PCB_TEMPERATURE: {
      ping1d_pcb_temperature response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_pcb_temperature(board.PCBTemperatureCC());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::VOLTAGE_5: {
      ping1d_voltage_5 response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_voltage_5(board.powerVoltage5MilliVolt());
      response.updateChecksum();
      return 0;
    }
    case Ping1dId::GOTO_BOOTLOADER: {
      board.goToBootLoader();
      return 0;
    }
    case Ping1dId::CONTINUOUS_START: {
      sonar.setIsModeContinuous(1);
      return 0;
    }
    case Ping1dId::CONTINUOUS_STOP: {
      sonar.setIsModeContinuous(0);
      return 0;
    }
    case Ping1dId::SET_OSS_PROFILE_CONFIGURATION: {
      ping1d_set_oss_profile_configuration request = reinterpret_cast<ping1d_set_oss_profile_configuration &>(msg);

      sonar.setNProfilePoints(request.number_of_points());
      sonar.setProfileNormalized(request.normalization_enabled());
      sonar.setProfileEnhanced(request.enhance_enabled());
      sonar.asyncRefresh();
      return 0;
    }
    case Ping1dId::OSS_PROFILE_CONFIGURATION: {
      ping1d_oss_profile_configuration response(_UARTTxBufferHead);
      uint16_t pending = decrementTxAvailable(response.msgDataLength());
      if (pending != 0) {
        return pending;
      }

      response.set_source_device_id(sonar.deviceID());
      response.set_number_of_points(sonar.nProfilePoints());
      response.set_normalization_enabled(sonar.isProfileNormalized());
      response.set_enhance_enabled(sonar.isProfileEnhanced());
      response.updateChecksum();
      return 0;
    }
    default:
      break;
  }
  return 0;
}

/**
 * ============================
 * Transmitters
 * ============================
 */

/**
 * @brief Updates the server auxiliary profile buffer with current Sonar device data from the scan.
 */
void SonarServer::updateMeasurement()
{
  PingSonar &sonar = PingSonar::GetInstance();

  _BufferProfile[0] = 'B';
  _BufferProfile[1] = 'R';
  _BufferProfile[6] = sonar.deviceID();
  _BufferProfile[7] = 0;

  /** Header Length 8 */
  reinterpret_cast<uint32_t&>(_BufferProfile[8]) = sonar.lockedDistance();
  reinterpret_cast<uint16_t&>(_BufferProfile[12]) = sonar.lockedConfidence();

  /** Estimative Length 14 */
  reinterpret_cast<uint16_t&>(_BufferProfile[14]) = sonar.transmitDuration();
  reinterpret_cast<uint32_t&>(_BufferProfile[16]) = sonar.pingNumber();
  reinterpret_cast<uint32_t&>(_BufferProfile[20]) = sonar.rangeScanStart();
  reinterpret_cast<uint32_t&>(_BufferProfile[24]) = sonar.rangeScanLength();
  reinterpret_cast<uint32_t&>(_BufferProfile[28]) = sonar.gainSetting();

  /** Parameters Length 32 */
  reinterpret_cast<uint16_t&>(_BufferProfile[32]) = sonar.nProfilePoints();
}

/**
 * @brief Transmits the sonar measurement stored in the auxiliary profile buffer.
 * @param type Final form to transmit the auxiliary profile.
 * @return Zero if the transmission was successful, otherwise the amount failed retries prior to this call.
 */
uint8_t SonarServer::transmitMeasurement(SonarMeasurementType type)
{
  if (_serverTxState != SonarServerTransmissionState::IDLE) {
    /** Let's go till overflow, if it happens, otherside will just accept the package loss :/ */
    return _profilesTxRetries++;
  }

  PingSonar &sonar = PingSonar::GetInstance();

  switch (type) {
    case SonarMeasurementType::DISTANCE_SIMPLE: {
      /** Message Size and ID */
      reinterpret_cast<uint16_t&>(_BufferProfile[2]) = static_cast<uint16_t>(5);
      reinterpret_cast<uint16_t&>(_BufferProfile[4]) = 1211;

      /** Converts confidence to uint8_t */
      _BufferProfile[11] = _BufferProfile[12];

      /** Header size 8 + Payload Size 5 = 13 */
      uint16_t check = computeCheckSum(_BufferProfile, 13U);
      reinterpret_cast<uint16_t&>(_BufferProfile[13]) = static_cast<uint16_t>(check);

      /** Trigger Transmission Header size 8 + Payload Size 5 + Checksum Size 2 = 15 */
      _serverTxState = SonarServerTransmissionState::PROFILE_TX_STARTED;
      HAL_UART_Transmit_DMA(&huart1, _BufferProfile, 15U);

      /** Returns confidence to uint16_t */
      _BufferProfile[12] = _BufferProfile[11];
      _BufferProfile[11] = 0U;
      break;
    }
    case SonarMeasurementType::DISTANCE: {
      /** Message Size and ID */
      reinterpret_cast<uint16_t&>(_BufferProfile[2]) = static_cast<uint16_t>(24);
      reinterpret_cast<uint16_t&>(_BufferProfile[4]) = 1212;

      /** Header size 8 + Payload Size 24 = 32 */
      uint16_t check = computeCheckSum(_BufferProfile, 32U);
      reinterpret_cast<uint16_t&>(_BufferProfile[32]) = static_cast<uint16_t>(check);

      /** Trigger Transmission Header size 8 + Payload Size 24 + Checksum Size 2 = 34 */
      _serverTxState = SonarServerTransmissionState::PROFILE_TX_STARTED;
      HAL_UART_Transmit_DMA(&huart1, _BufferProfile, 34U);
      break;
    }
    case SonarMeasurementType::PROFILE: {
      uint16_t nPoints = sonar.nProfilePoints();

      reinterpret_cast<uint16_t&>(_BufferProfile[2]) = static_cast<uint16_t>(26 + nPoints);
      reinterpret_cast<uint16_t&>(_BufferProfile[4]) = 1300;

      /** Header size 8 + Payload Size 26 + N points = 32 */
      uint16_t check = computeCheckSum(_BufferProfile, 34U + nPoints);
      reinterpret_cast<uint16_t&>(_BufferProfile[34U + nPoints]) = static_cast<uint16_t>(check);

      /** Trigger Transmission Header size 8 + Payload Size 26 + N points + Checksum Size 2 = 36 + N points */
      _serverTxState = SonarServerTransmissionState::PROFILE_TX_STARTED;
      HAL_UART_Transmit_DMA(&huart1, _BufferProfile, 36U + nPoints);
      break;
    }
    default:
      break;
  }

  /** Make sure to clear informing users that transmission was successful */
  _profilesTxRetries = 0U;
  return _profilesTxRetries;
}

/**
 * ============================
 * Utils
 * ============================
 */

/**
 * @brief Computes the sum based checksum for a given buffer.
 * @param buffer The buffer to compute the checksum for.
 * @param size The size of the buffer.
 * @return The computed checksum.
 */
uint16_t SonarServer::computeCheckSum(uint8_t *buffer, uint16_t size)
{
  uint16_t check = 0;
  for(uint32_t i = 0; i < size; ++i) {
      check = static_cast<uint16_t>(buffer[i] + check);
  }
  return check;
}

/**
 * ============================
 * Getters / Setters
 * ============================
 */

/**
 * @brief Set the _serverTxState
 *
 * @param state The value to set.
 */
void SonarServer::setServerTxState(SonarServerTransmissionState state)
{
  _serverTxState = state;
}

/**
 * @brief Set the _UARTTxBufferAvailable
 *
 * @param available The value to set.
 */
void SonarServer::setTxAvailable(uint16_t available)
{
  _UARTTxBufferAvailable = available;
}

/**
 * @brief Set the _UARTRxBufferReceived
 *
 * @param received The value to set.
 */
void SonarServer::setRxReceived(uint16_t received)
{
  _UARTRxBufferReceived = received;
}

/**
 * @brief Set the _UARTRxBufferParsed
 *
 * @param received The value to set.
 */
void SonarServer::setRxParsed(uint16_t received)
{
  _UARTRxBufferParsed = received;
}

/**
 * @brief Used to inform server that a certain amount of bytes have been used from the TX buffer.
 * @param decrement The amount to decrement.
 * @return Zero if the decrement was successful, otherwise the amount that couldn't be decremented.
 */
uint16_t SonarServer::decrementTxAvailable(uint16_t decrement)
{
  if (decrement > _UARTTxBufferAvailable) {
    return decrement;
  }

  _UARTTxBufferAvailable -= decrement;
  _UARTTxBufferHead += decrement;

  return 0U;
}

/**
 * ============================
 * Interruptions
 * ============================
 */

/** TX */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /** We only care about UART1 */
  if (huart != &huart1) {
    return;
  }

  SonarServer &server = SonarServer::GetInstance();
  SonarServerTransmissionState state = server.serverTxState();

  if (state == SonarServerTransmissionState::SERVER_TX_HALF_COMPLETE) {
    server.setTxAvailable(UART_TX_BUFFER_SIZE);
  }

  server.setServerTxState(SonarServerTransmissionState::IDLE);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  /** We only care about UART1 */
  if (huart != &huart1) {
    return;
  }

  SonarServer &server = SonarServer::GetInstance();
  SonarServerTransmissionState state = server.serverTxState();

  if (state == SonarServerTransmissionState::SERVER_TX_STARTED) {
    server.setServerTxState(SonarServerTransmissionState::SERVER_TX_HALF_COMPLETE);
  } else {
    server.setServerTxState(SonarServerTransmissionState::PROFILE_TX_HALF_COMPLETE);
  }
}

/** RX */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /** We only care about UART1 */
  if (huart != &huart1) {
    return;
  }

  SonarServer &server = SonarServer::GetInstance();
  server.setRxReceived(Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /** We only care about UART1 */
  if (huart != &huart1) {
    return;
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) {
    __HAL_UART_CLEAR_FEFLAG(huart);
  }

  /** TODO: If a frame error occurs, enable auto baudrate on 0x55 */

  SonarServer &server = SonarServer::GetInstance();
  server.init();
}
