#pragma once

#include <cstdint>

#include "ping-message-common.h"
#include "ping-message-ping1d.h"
#include "ping-parser.h"

#include "config.h"
#include "Sonar/sonar.h"

/**
 * @enum SonarServerTransmissionState
 * @brief Represents the transmission state of the Sonar server UART TX.
 */
enum SonarServerTransmissionState : uint8_t
{
  IDLE = 0,                 /**< The server is idle and not transmitting data. TX DMA is free to use */
  SERVER_TX_STARTED,        /**< Data transmission by the server has started. */
  SERVER_TX_HALF_COMPLETE,  /**< Data transmission by the server is halfway complete. */
  PROFILE_TX_STARTED,       /**< Auxiliary profile buffer data transmission has started. */
  PROFILE_TX_HALF_COMPLETE, /**< Auxiliary profile buffer data transmission is halfway complete. */
};


class SonarServer
{
protected:
  SonarServer() = default;

public:
  static SonarServer& GetInstance();

  SonarServer(const SonarServer &other) = delete;
  void operator=(const SonarServer &) = delete;

public:
  void init();
  void update();
  uint8_t router(ping_message &msg);


  void updateMeasurement();
  uint8_t transmitMeasurement(SonarMeasurementType type);
  uint8_t sendNackMessage(const char* message, uint16_t message_id);

public:
  void awaitServerTxState(SonarServerTransmissionState state, uint16_t retries);
  uint16_t decrementTxAvailable(uint16_t decrement);
  void incrementTxAvailable(uint16_t increment);

  /** Getters / Setters */

  uint8_t *UARTBufferRx() { return _UARTBufferRx; }
  uint8_t *UARTBufferTx() { return _UARTBufferTx; }
  uint8_t *bufferProfile() { return _BufferProfile; }

  void setServerTxState(SonarServerTransmissionState state);
  SonarServerTransmissionState serverTxState() const { return _serverTxState; }

  void setTxAvailable(uint16_t available);
  uint16_t txAvailable() const { return _UARTTxBufferAvailable; }

  void setRxReceived(uint16_t received);
  uint16_t rxReceived() const { return _UARTRxBufferReceived; }

  void setRxParsed(uint16_t received);
  uint16_t rxParsed() const { return _UARTRxBufferParsed; }

protected:
  void processRxBuffer();
  void processTxBuffer();

  /** Utils */

  uint16_t computeCheckSum(uint8_t *buffer, uint16_t size);

private:
  /** Server State Machine control */

  SonarServerTransmissionState _serverTxState = SonarServerTransmissionState::IDLE; /**< Current transmission state. */

  uint16_t _pendingTxSize = 0U;     /**< In case a message is parsed but it's response does not fit available TX */
  uint8_t _profilesTxRetries = 0U;  /**< When using non blocking profile transmission how many times couldn't lock */

  /** Server TX control */

  uint8_t *_UARTTxBufferHead = _UARTBufferTx;             /**< Pointer to the head of the TX buffer. */
  uint16_t _UARTTxBufferAvailable = UART_TX_BUFFER_SIZE;  /**< Available amount of bytes to be written to TX buffer */

  /** Server RX control */

  uint16_t _UARTRxBufferParsed = 0U;    /**< Current point of the RX head that the parser have consumed */
  uint16_t _UARTRxBufferReceived = 0U;  /**< Current point of the RX head where teh DMA is storing */

  /** Server Memory Buffers */

  uint8_t _UARTBufferRx[UART_RX_BUFFER_SIZE];             /**< Buffer for received UART data. */
  uint8_t _UARTBufferTx[UART_TX_BUFFER_SIZE];             /**< Buffer for transmitted UART data. */
  uint8_t _BufferProfile[36U + PROFILE_MSG_BUFFER_SIZE];  /**< Auxiliary profile storage for fast scan / transmit */
};
