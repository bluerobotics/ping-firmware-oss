#include <cstdint>

#include "stm32f3xx_ll_usart.h"
#include "usart.h"

#include "config.h"
#include "Sonar/board.h"
#include "Sonar/server.h"

#define PREPARE_ACK_REQUEST(message_type)                            \
  AckResponseHandler handler(_UARTTxBufferHead, msg.message_id());   \
  if (handler.pending() != 0) {                                      \
    return handler.pending();                                        \
  }                                                                  \
  auto &request = reinterpret_cast<message_type&>(msg);

#define ABORT_ACK_REQUEST(message)                                   \
  handler.abort(message);

#define FINISH_RESPONSE()                                            \
  response.updateChecksum();

#define PREPARE_RESPONSE(message_type)                               \
  message_type response(_UARTTxBufferHead);                          \
  uint16_t pending = decrementTxAvailable(response.msgDataLength()); \
  if (pending != 0) {                                                \
    return pending;                                                  \
  }                                                                  \
  response.set_source_device_id(sonar.deviceID());

#define FINISH_RESPONSE()                                            \
  response.updateChecksum();


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
    _UARTRxBufferParsed = 1U + (_UARTRxBufferParsed) % UART_RX_BUFFER_SIZE;
    PingParser::State parse_result = pingParser.parseByte(_UARTBufferRx[_UARTRxBufferParsed - 1U]);

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
      PREPARE_RESPONSE(common_protocol_version);
      response.set_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_version_minor(PROTOCOL_VERSION_MINOR);
      response.set_version_patch(PROTOCOL_VERSION_PATCH);
      FINISH_RESPONSE();
      return 0;
    }
    case CommonId::DEVICE_INFORMATION: {
      PREPARE_RESPONSE(common_device_information);
      response.set_device_type(PING_DEVICE_TYPE);
      response.set_device_revision(PING_DEVICE_REVISION);
      response.set_firmware_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_firmware_version_minor(PROTOCOL_VERSION_MINOR);
      response.set_firmware_version_patch(PROTOCOL_VERSION_PATCH);
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::FIRMWARE_VERSION: {
      PREPARE_RESPONSE(ping1d_firmware_version);
      response.set_device_type(PING_DEVICE_TYPE);
      response.set_device_model(PING_DEVICE_MODEL);
      response.set_firmware_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_firmware_version_minor(PROTOCOL_VERSION_MINOR);
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::GENERAL_INFO: {
      PREPARE_RESPONSE(ping1d_general_info);
      response.set_firmware_version_major(PROTOCOL_VERSION_MAJOR);
      response.set_firmware_version_minor(PROTOCOL_VERSION_MINOR);
      response.set_voltage_5(board.powerVoltage5MilliVolt());
      response.set_ping_interval(sonar.pingInterval());
      response.set_gain_setting(sonar.gainSetting());
      response.set_mode_auto(sonar.isModeAuto());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_DEVICE_ID: {
      PREPARE_ACK_REQUEST(ping1d_set_device_id);
      sonar.setDeviceID(request.device_id());
      return 0;
    }
    case Ping1dId::DEVICE_ID: {
      PREPARE_RESPONSE(ping1d_device_id);
      response.set_source_device_id(sonar.deviceID());
      response.set_device_id(sonar.deviceID());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_RANGE: {
      PREPARE_ACK_REQUEST(ping1d_set_range);
      const auto scan_start = sonar.rangeScanStart();
      sonar.setRangeScanStart(request.scan_start());
      if (sonar.setRangeScanLength(request.scan_length()) != HAL_OK) {
        sonar.setRangeScanStart(scan_start);
        return ABORT_ACK_REQUEST("Range scan length out of range");
      }
      return 0;
    }
    case Ping1dId::RANGE: {
      PREPARE_RESPONSE(ping1d_range);
      response.set_scan_start(sonar.rangeScanStart());
      response.set_scan_length(sonar.rangeScanLength());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_SPEED_OF_SOUND: {
      PREPARE_ACK_REQUEST(ping1d_set_speed_of_sound);
      sonar.setSpeedOfSound(request.speed_of_sound());
      return 0;
    }
    case Ping1dId::SPEED_OF_SOUND: {
      PREPARE_RESPONSE(ping1d_speed_of_sound);
      response.set_speed_of_sound(sonar.speedOfSound());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_MODE_AUTO: {
      PREPARE_ACK_REQUEST(ping1d_set_mode_auto);
      sonar.setIsModeAuto(request.mode_auto());
      return 0;
    }
    case Ping1dId::MODE_AUTO: {
      PREPARE_RESPONSE(ping1d_mode_auto);
      response.set_mode_auto(sonar.isModeAuto());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_PING_INTERVAL: {
      PREPARE_ACK_REQUEST(ping1d_set_ping_interval);
      sonar.setPingInterval(request.ping_interval());
      break;
    }
    case Ping1dId::PING_INTERVAL: {
      PREPARE_RESPONSE(ping1d_ping_interval);
      response.set_ping_interval(sonar.pingInterval());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_GAIN_SETTING: {
      PREPARE_ACK_REQUEST(ping1d_set_gain_setting);
      if (sonar.setGainSetting(request.gain_setting()) != HAL_OK) {
        return ABORT_ACK_REQUEST("Gain setting out of range");
      }
      return 0;
    }
    case Ping1dId::GAIN_SETTING: {
      PREPARE_RESPONSE(ping1d_gain_setting);
      response.set_gain_setting(sonar.gainSetting());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::SET_PING_ENABLE: {
      PREPARE_ACK_REQUEST(ping1d_set_ping_enable);
      sonar.setIsPingEnabled(request.ping_enabled());
      return 0;
    }
    case Ping1dId::PING_ENABLE: {
      PREPARE_RESPONSE(ping1d_ping_enable);
      response.set_ping_enabled(sonar.isPingEnabled());
      FINISH_RESPONSE();
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
      PREPARE_RESPONSE(ping1d_transmit_duration);
      response.set_transmit_duration(sonar.transmitDuration());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::PROCESSOR_TEMPERATURE: {
      PREPARE_RESPONSE(ping1d_processor_temperature);
      response.set_processor_temperature(board.processorTemperatureCC());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::PCB_TEMPERATURE: {
      PREPARE_RESPONSE(ping1d_pcb_temperature);
      response.set_pcb_temperature(board.PCBTemperatureCC());
      FINISH_RESPONSE();
      return 0;
    }
    case Ping1dId::VOLTAGE_5: {
      PREPARE_RESPONSE(ping1d_voltage_5);
      response.set_voltage_5(board.powerVoltage5MilliVolt());
      FINISH_RESPONSE();
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
      PREPARE_ACK_REQUEST(ping1d_set_oss_profile_configuration);
      sonar.setNProfilePoints(request.number_of_points());
      sonar.setProfileNormalized(request.normalization_enabled());
      sonar.setProfileEnhanced(request.enhance_enabled());
      return 0;
    }
    case Ping1dId::OSS_PROFILE_CONFIGURATION: {
      PREPARE_RESPONSE(ping1d_oss_profile_configuration);
      response.set_number_of_points(sonar.nProfilePoints());
      response.set_normalization_enabled(sonar.isProfileNormalized());
      response.set_enhance_enabled(sonar.isProfileEnhanced());
      FINISH_RESPONSE();
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
      static ping1d_distance_simple distance_simple;
      distance_simple.set_source_device_id(sonar.deviceID());
      distance_simple.set_destination_device_id(0);
      distance_simple.set_distance(sonar.lockedDistance());
      distance_simple.set_confidence(sonar.lockedConfidence());
      distance_simple.updateChecksum();
      HAL_UART_Transmit_DMA(&huart1, distance_simple.msgData, distance_simple.msgDataLength());
      break;
    }
    case SonarMeasurementType::DISTANCE: {
      static ping1d_distance distance;
      distance.set_source_device_id(sonar.deviceID());
      distance.set_destination_device_id(0);
      distance.set_distance(sonar.lockedDistance());
      distance.set_confidence(sonar.lockedConfidence());
      distance.set_transmit_duration(sonar.transmitDuration());
      distance.set_ping_number(sonar.pingNumber());
      distance.set_scan_start(sonar.rangeScanStart());
      distance.set_scan_length(sonar.rangeScanLength());
      distance.set_gain_setting(sonar.gainSetting());
      distance.updateChecksum();
      HAL_UART_Transmit_DMA(&huart1, distance.msgData, distance.msgDataLength());
      break;
    }
    case SonarMeasurementType::PROFILE: {
      ping1d_profile profile(_BufferProfile, sonar.nProfilePoints());
      profile.set_source_device_id(sonar.deviceID());
      profile.set_destination_device_id(0);
      profile.set_distance(sonar.lockedDistance());
      profile.set_confidence(sonar.lockedConfidence());
      profile.set_transmit_duration(sonar.transmitDuration());
      profile.set_ping_number(sonar.pingNumber());
      profile.set_scan_start(sonar.rangeScanStart());
      profile.set_scan_length(sonar.rangeScanLength());
      profile.set_gain_setting(sonar.gainSetting());
      profile.set_profile_data_length(sonar.nProfilePoints());
      // Pointes are already filled by the sonar class using bufferProfileData
      profile.updateChecksum();
      HAL_UART_Transmit_DMA(&huart1, profile.msgData, profile.msgDataLength());
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
 * @brief Creates and transmits a NACK message with the given text
 * @param message The message to include in the NACK
 * @param message_id The ID of the message being NACKed
 * @return Zero if successful, otherwise the amount of bytes pending to be written
 */
uint8_t SonarServer::sendNackMessage(const char* message, uint16_t message_id)
{
  const auto message_length = strlen(message);
  common_nack response_nack(_UARTTxBufferHead, message_length);
  uint16_t pending_nack = decrementTxAvailable(response_nack.msgDataLength());
  if (pending_nack != 0) {
      return pending_nack;
  }

  for (uint16_t i = 0; i < message_length; i++) {
      response_nack.set_nack_message_at(i, message[i]);
  }
  response_nack.set_source_device_id(PingSonar::GetInstance().deviceID());
  response_nack.set_nacked_id(message_id);
  response_nack.updateChecksum();
  return 0;
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
 * @brief Used to inform server that a certain amount of bytes have been freed from the TX buffer.
 * @param increment The amount to increment.
 */
void SonarServer::incrementTxAvailable(uint16_t increment)
{
  if (increment + _UARTTxBufferAvailable > UART_TX_BUFFER_SIZE) {
    increment = UART_TX_BUFFER_SIZE - _UARTTxBufferAvailable;
  }

  _UARTTxBufferAvailable += increment;
  _UARTTxBufferHead -= increment;
}

/**
 * @brief Handles the response to a message, sending an ack or nack based on the result of the operation.
 * @param txBufferHead The buffer to write the response to.
 * @param message The message to respond to.
 */
AckResponseHandler::AckResponseHandler(uint8_t* txBufferHead, uint16_t message_id)
  : _response(txBufferHead), _message_id(message_id), _aborted(false), _pending(0)
{
  _pending = SonarServer::GetInstance().decrementTxAvailable(_response.msgDataLength());
}

/**
 * @brief Destructor for the AckResponseHandler class, responsible for sending an ack, if necessary.
 */
AckResponseHandler::~AckResponseHandler() {
  if (_aborted || _pending != 0) {
    return;
  }
  _response.set_source_device_id(PingSonar::GetInstance().deviceID());
  _response.set_acked_id(_message_id);
  _response.updateChecksum();
}

/**
 * @brief Get the pending amount of bytes.
 * @return The pending amount of bytes.
 */
uint16_t AckResponseHandler::pending() const {
  return _pending;
}

/**
 * @brief Aborts the response to a message.
 * @param errorMessage The error message to send.
 * @return The amount of bytes pending to be written.
 */
uint16_t AckResponseHandler::abort(const char* errorMessage) {
  _aborted = true;
  SonarServer::GetInstance().incrementTxAvailable(_response.msgDataLength());
  return SonarServer::GetInstance().sendNackMessage(errorMessage, _message_id);
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
