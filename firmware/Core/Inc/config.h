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
#define ADC4_DMA_BUFFER_SIZE (61200U - PROFILE_MSG_BUFFER_SIZE)

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
#define DESIRED_RANGE_RESOLUTION 0.048f

/**
 * @def MIN_TRANSMIT_REPETITIONS
 * @brief Defines the minimum number of transmit repetitions to ensure a reliable signal.
 */
#define MIN_TRANSMIT_REPETITIONS 3U

/**
 * @def MIN_TRANSMIT_DURATION_US
 * @brief Defines the maximum number of points used when calculating the avg to adjust the signal bias using DAC.
 */
#define DAC_BIAS_ADJUST_MEAN_WINDOW_SIZE 8 * 1000U

/**
 * @def DESIRED_SIGNAL_CENTER_VALUE
 * @brief Defines the desired center value of the signal.
 */
#define DESIRED_SIGNAL_CENTER_VALUE 127

/**
 * ============================
 * Auto Range Control
 * ============================
 */

/**
 * @def AUTO_RANGE_SWEEP_START_MM
 * @brief Defines the starting distance (in millimeters) for the auto-ranging sweep.
 *
 * This value sets the minimum scan range limit where auto-ranging begins detecting targets.
 */
#define AUTO_RANGE_SWEEP_START_MM 5000U

/**
 * @def AUTO_RANGE_SWEEP_END_MM
 * @brief Defines the ending distance (in millimeters) for the auto-ranging sweep.
 *
 * This value sets the maximum scan range limit where auto-ranging stops searching for targets.
 */
#define AUTO_RANGE_SWEEP_END_MM 70000U

/**
 * @def AUTO_RANGE_SWEEP_STEP_MM
 * @brief Defines the step size (in millimeters) for the auto-ranging sweep.
 */
#define AUTO_RANGE_SWEEP_STEP_MM 20000U

/**
 * @def AUTO_RANGE_CONFIDENCE_THRESHOLD
 * @brief Specifies the confidence threshold for auto-ranging to trigger.
 *
 * @note: Percentage in format 0U to 100U.
 *
 */
#define AUTO_RANGE_CONFIDENCE_THRESHOLD 50U

/**
 * @def AUTO_RANGE_TARGET_CENTER_AT
 * @brief Specifies the target position within the scan range for auto-ranging.
 *
 * @note: Percentage in format 0.0f to 1.0f.
 *
 * This value represents a percentage of the total scan range where the main target
 * is expected to be centered. For example, a value of `0.7f` means the target should
 * be positioned at 70% of the scan range from the starting point.
 */
#define AUTO_RANGE_TARGET_CENTER_AT 0.7f

/**
 * @def AUTO_RANGE_BOUNDARY_DELTA
 * @brief Defines the margin around the target center for auto-ranging boundaries.
 *
 * @note: Percentage in format 0.0f to 1.0f.
 *
 * This value determines the range expansion around the expected target center,
 * setting lower and upper limits for auto-ranging. A higher value increases
 * the tolerance around the target, allowing for more variation in its position without
 * retriggering the auto-ranging process.
 */
#define AUTO_RANGE_BOUNDARY_DELTA 0.2f

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
