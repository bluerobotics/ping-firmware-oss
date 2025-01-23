#pragma once

/**
 * ============================
 * General
 * ============================
 */

/**
 * @def IWDG_REFRESH_INTERVAL_MS
 * @brief Defines the interval in milliseconds at which the IWDG is refreshed.
 */
#define IWDG_REFRESH_INTERVAL_MS 100U

/**
 * ============================
 * Device Version Control
 * ============================
 */

/**
 * @def PING_DEVICE_TYPE
 * @brief Defines the type of the Ping device.
 */
#define PING_DEVICE_TYPE 9U

/**
 * @def PING_DEVICE_MODEL
 * @brief Defines the model of the Ping device.
 */
#define PING_DEVICE_MODEL 1U

/** @def PING_DEVICE_REVISION
 *  @brief Defines the hardware revision of the Ping device.
 */
#define PING_DEVICE_REVISION 19U

/**
 * ============================
 * Protocol Version Control
 * ============================
 */

/**
 * @def PROTOCOL_VERSION_MAJOR
 * @brief Defines the major version of the communication protocol.
 */
#define PROTOCOL_VERSION_MAJOR 0U

/**
 * @def PROTOCOL_VERSION_MINOR
 * @brief Defines the minor version of the communication protocol.
 */
#define PROTOCOL_VERSION_MINOR 0U

/**
 * @def PROTOCOL_VERSION_PATCH
 * @brief Defines the patch version of the communication protocol.
 */
#define PROTOCOL_VERSION_PATCH 0U

/**
 * ============================
 * UART Server Control
 * ============================
 */

/**
 * @def SINGLE_MESSAGE_SIZE
 * @brief Specifies usual max single message size received in UART server in bytes.
 */
#define SINGLE_MESSAGE_SIZE 18U

/**
 * @def UART_RX_BUFFER_SIZE
 * @brief Defines the size of the UART receive buffer in bytes.
 */
#define UART_RX_BUFFER_SIZE 256U

/**
 * @def UART_TX_BUFFER_SIZE
 * @brief Defines the size of the UART transmit buffer in bytes.
 */
#define UART_TX_BUFFER_SIZE 256U

/**
 * @def PROFILE_MSG_BUFFER_SIZE
 * @brief Defines the size of auxiliary profile data generated in the scan for fast scan / transmit modes.
 */
#define PROFILE_MSG_BUFFER_SIZE 200U

/**
 * ============================
 * DMA Buffers
 * ============================
 */

/**
 * @def ADC1_DMA_BUFFER_SIZE
 * @brief Size of the ADC1 (Board 5V, PCB Temperature, Processor Temperature) DMA buffer in bytes.
 */
#define ADC1_DMA_BUFFER_SIZE 3U

/**
 * @def ADC4_DMA_BUFFER_SIZE
 * @brief Defines the size of the ADC4 DMA buffer in bytes, used for storing echo return signal samples.
 *
 * @note This buffer typically occupies a significant portion of the available RAM.
 *       Use caution when modifying this value to avoid memory issues.
 */
#define ADC4_DMA_BUFFER_SIZE (61200 - PROFILE_MSG_BUFFER_SIZE)

/**
 * ============================
 * Drive / Sampling Frequency Hardware Adjusting
 * ============================
 */

/**
 * @def DRIVE_FREQUENCY_HZ
 * @brief Defines the frequency of the Piezo drive signal in hertz.
 */
#define DRIVE_FREQUENCY_HZ 115015U

/**
 * @def MIN_SAMPLING_FREQUENCY_HZ
 * @brief Defines the minimum allowed sampling frequency in hertz.
 *
 * @note It is recommended to use integer multiples of the drive frequency.
 */
#define MIN_SAMPLING_FREQUENCY_HZ (4U * DRIVE_FREQUENCY_HZ)

/**
 * @def MAX_SAMPLING_FREQUENCY_HZ
 * @brief Defines the maximum allowed sampling frequency in hertz.
 *
 * @note It is recommended to use integer multiples of the drive frequency.
 * @note This value can go up to 3.6 MHz for high precision, but exceeding 1 MHz is not recommended.
 *       At higher rates, the sonar's hardware cannot charge the ADC capacitor fast enough to sample
 *       correctly, effectively inducing a low-pass filter in the signal.
 */
#define MAX_SAMPLING_FREQUENCY_HZ (8U * DRIVE_FREQUENCY_HZ)

/**
 * @def DESIRED_RANGE_RESOLUTION
 * @brief Defines the desired range resolution as percentage of the scan range.
 */
#define DESIRED_RANGE_RESOLUTION 0.005f

/**
 * @def MIN_TRANSMIT_REPETITIONS
 * @brief Defines the minimum number of transmit repetitions to ensure a reliable signal.
 */
#define MIN_TRANSMIT_REPETITIONS 4U

/**
 * ============================
 * DSP Algorithm Configuration
 * ============================
 */

/**
 * @def ECHO_FINDER_WINDOW_SIZE
 * @brief Defines the size of the window used by the echo finder algorithm.
 */
#define ECHO_FINDER_WINDOW_SIZE 16U

/**
 * @def ECHO_FINDER_THRESHOLD
 * @brief Defines the threshold used by the echo finder algorithm.
 */
#define ECHO_FINDER_THRESHOLD 15U
