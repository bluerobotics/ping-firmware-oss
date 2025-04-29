# Open Source Sonar (OSS) Firmware for Ping1D Echosounder Devices

## Contents

1. [Context and inspiration](#context-and-inspiration)
1. [Current functionality](#current-functionality)
1. [Installation](#installation)
1. [Standard operation](#standard-operation)
   1. [Filtering and processing](#filtering-and-processing)
   1. [Distance estimation](#distance-estimation)
   1. [Communication and interfacing](#communication-and-interfacing)
1. [Reference hardware](#reference-hardware)
   1. [Known compatible devices](#known-compatible-devices)
   1. [Main control connections](#main-control-connections)
   1. [Amplification and filtering (Analog Sector)](#amplification-and-filtering-analog-sector)
   1. [Sensing](#sensing)
1. [Development and contributions](#development-and-contributions)
   1. [Firmware overview](#firmware-overview)
   1. [Configuration](#configuration)
   1. [Dependencies](#dependencies)
   1. [Building the firmware](#building-the-firmware)
   1. [Flashing the device](#flashing-the-device)
1. [Project history](#project-history)


## Context and inspiration

When exploring and measuring the underwater environment, [sonar technologies](https://bluerobotics.com/learn/a-smooth-operators-guide-to-underwater-sonars-and-acoustic-devices/) are invaluable, and just plain cool!

Recent advancements have made sonar devices cheaper and more accessible, but proprietary hardware and software designs mean they're generally still hard to play and experiment with, and research applications like custom algorithms and transducer designs are difficult to set up, test, and compare. While we can dream of a world of open source sonar devices, even a single transducer can enable many use-cases, so let's start there!

> As a few examples from the Blue Robotics forums, consider possibilities like:
> - [Acoustic beacons](https://discuss.bluerobotics.com/t/recovering-research-equipment-after-6months-with-bluerov2/12299/2) for equipment recovery
> - Exploring [acoustic positioning](https://discuss.bluerobotics.com/t/best-way-to-determine-distance-between-multiple-bluerovs/8474/2)
> - [Measuring acoustic attenuation of the environment](https://discuss.bluerobotics.com/t/raw-profile-data-from-ping1d/12425/6)
> - [Oceanographic water property sensing](https://discuss.bluerobotics.com/t/raw-profile-data-from-ping1d/12425/5)
> - [Basic scanning sonar](https://discuss.bluerobotics.com/t/using-ping1d-and-servo-with-arduino-nano-every/11715)
> - [Echosounder distance tracking algorithms](https://discuss.bluerobotics.com/t/interpretation-of-sonar-raw-data/11722/4)
> - Anti-synchronisation for [obstacle avoidance](https://discuss.bluerobotics.com/t/suitability-of-ping-sonar-in-caves/11655)

Developing useful devices and algorithms takes a lot of work, so it makes sense to limit the initial project scope by building upon existing projects and standards. As a starting point, there are existing open communication protocols available for some common use-cases, and using them can provide guidelines for functionality that should be implemented, while also allowing for expansion as new use-cases are explored, and connecting to existing display/control software that's compatible with those protocols. There are also purchaseable devices that already solve many of the physical challenges of operating sonar underwater, which could have their existing functionalities substantially expanded by an open source, general purpose sonar firmware.


## Current functionality

> 💡 **This project is intended as a learning resource**, and as an entryway for playing with / testing sonar technologies.

> ⚠️ Interfacing with existing device electronics was [done through reverse engineering](#project-history), so cannot be guaranteed to be correct/identical.
>
> Without affiliation with the original device designers or developers, outputs from this project cannot be recognised as official replacement firmware for devices it happens to be compatible with.

Implemented features include:
- Acoustic mono-frequency pulse transmission
   - e.g. for a beacon/pinger
- Acoustic receiving
   - e.g. for a hydrophone / for testing beam width of other sonars
- [Basic signal processing](#filtering-and-processing), to improve data scaling and interpretability
- Sonar and echosounding
   - [Distance estimation](#distance-estimation), for surface/target tracking or obstacle avoidance
   - Compatibility with [`ping-protocol`'s echosounding messages](#communication-and-interfacing), implemented on an STM32 microcontroller


## Installation

> 💡 Pre-built firmware binaries are available in the assets of the [releases](https://github.com/bluerobotics/ping-firmware-oss/releases), and as artifacts of automated [actions](https://github.com/bluerobotics/ping-firmware-oss/actions) that run when code is added to the repository or submitted as a pull request.
 
Installation can typically be performed [using Ping Viewer](https://docs.bluerobotics.com/ping-viewer/firmware-update/#manual-firmware-update), or more directly [using the stm32flash tool](https://docs.bluerobotics.com/ping-viewer/firmware-update/#ping-sonar-device-recovery). There are more details in the [flashing the device](#flashing-the-device) section.


## Standard operation

After relevant setup of the hardware, the firmware is responsible for transmitting pulses, receiving and processing the echo signals (after an optional delay), and sending processed data to the host computer. A more detailed breakdown is included in the [Firmware Overview](#firmware-overview) section.

### Filtering and processing

After any [electrical filtering provided by the hardware](#amplification-and-filtering-analog-sector), the measured sound signals are [processed](https://github.com/search?q=repo%3Abluerobotics%2Fping-firmware-oss+void+PingSonar%3A%3AprocessProfile+path%3Asonar.cpp&type=code) into squared sequential differences, then normalised, before the full-rate samples are sub-sampled (by maximum-value selection) into the number of points being communicated in a profile.

> 💡 In future it makes sense to add a software-based frequency filter, to prioritise sound samples that match a target frequency (e.g. the frequency of the transmitted pulses, to improve echo detection).

### Distance estimation

Distances are estimated using a [peak detection algorithm](https://github.com/ES-Alexander/ping-firmware-oss/blob/the-easy-thing/firmware/Core/Src/DSP/echo_finder.c) that runs on the measured samples (prior to profile sub-sampling), after finding and excluding the device's resonant ringing period after a transmit pulse.

### Communication and interfacing

Full-compatibility is provided with the [`common`](https://docs.bluerobotics.com/ping-protocol/pingmessage-common/) and [`ping1d`](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/) message sets of the open-source [Ping Protocol](https://docs.bluerobotics.com/ping-protocol/) for sonar devices.

As a result, sonars running the firmware can be readily interfaced with the following:
- **Applications:**
   - (Blue Robotics) [Ping Viewer](https://docs.bluerobotics.com/ping-viewer)
   - (Cerulean) [SonarView](https://ceruleansonar.com/sonarview/)
- **Devices:**
   - flight controller boards running ArduSub / ArduRover autopilots, [as a rangefinder](https://ardupilot.org/rover/docs/common-bluerobotics-ping.html)
- **Software libraries:**
   - [Arduino](https://github.com/bluerobotics/ping-arduino)
   - [C++](https://github.com/bluerobotics/ping-cpp)
   - [Python](https://github.com/bluerobotics/ping-python)
   - [Rust](https://github.com/bluerobotics/ping-rs)


## Reference hardware

A viable hardware example has been determined by reverse-engineering a Blue Robotics Ping Sonar PCB** into the following high level schematic:
<img src="./docs/high_level.svg" alt="High-level example hardware diagram, from the Blue Robotics Ping Sonar" width="100%">

> ****NOTE:** The Ping Sonar is known to work for at least echosounding purposes (as that is what it is sold as), but general design tradeoffs are not commented on as they were not the focus of [the reverse-engineering analysis](#project-history).

### Known compatible devices
- [Original Ping Sonar](https://web.archive.org/web/20230330171011/https://bluerobotics.com/store/sensors-sonars-cameras/sonar/ping-sonar-r2-rp/) from Blue Robotics
    - Advertised with a 30 degree beamwidth, 115kHz transducer frequency, 300m depth rating, and 0.5-50m range*
- [Second generation Ping Sonar](https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/) from Blue Robotics
    - Advertised with a 25 degree beamwidth, 115kHz transducer frequency, 300m depth rating, and 0.3-100m range*
    - **PCB [used as a reverse-engineering target](#project-history) when developing this firmware**

> ***NOTE:** Advertised usable range is indicative of hardware capabilities - matching values cannot be guaranteed with this open source firmware because the official firmware uses proprietary (i.e. unknown) processing and estimation algorithms

### Main control connections

The reference hardware uses an [STM32F303RET6](https://www.st.com/resource/en/datasheet/stm32f303re.pdf) MCU (microcontroller) from STMicroelectronics, which manages all key functionalities of the sonar, including:

- Generating the sonar pulse (on pin `PC6`)
   - Achieved in this firmware using a PWM (Pulse-Width Modulation) output, with its signal generated by **Advanced Timer 8 (`TIM8`)** on **Channel 1**
- Receiving the echo signal (on pin `PB15`)
   - Captured using **ADC4 Channel 5**, which is a Fast Channel
- Processing the received signal

Communication between the MCU and the host computer occurs via a serial interface.

### Amplification and filtering (Analog Sector)

The amplification and filtering of the received signal are proprietary and have been kept closed-source to avoid any potential violation of the sonar's intellectual property rights. The only information that this document shares is that the received signal is filtered using a band-pass filter centered around the transmission frequency of **115 kHz**.

For the firmware, it is important to note that the analog sector shares some functionality with the internal operational amplifiers (OPAMPs) of the MCU. The following details are relevant:

- **OPAMPs 2 and 3:** Configured in PGA (Programmable Gain Amplifier) mode for gain control.
- **OPAMP 4:** Configured in standalone mode for bias (offset adjustment), working in conjunction with the internal DAC.

The pin configurations for these OPAMPs are available in the [high-level schematic](#reference-hardware).

### Sensing

The sonar board is equipped with sensing capabilities that allow some measurements to be performed. The available sensing points include:

- **PCB Temperature Sensing:** Available on `PA2` (**ADC1 Channel 3**).
- **Supply Voltage (5V) Sensing:** Available on `PC2` (**ADC1 Channel 8**).
- **Internal Processor Temperature Sensing:** Accessible via the **ADC1 Temperature Sensor channel**.


## Development and contributions

> 💡 **Code is provided as-is** (under an [MIT license](/LICENSE)), and not actively _supported_, but pull requests with improvements and new features are welcome.

The firmware is developed using the STM32 CubeMX tool for initialization and configuration. It is written in C++, and is compiled using CMake and the GNU Arm Embedded Toolchain.

### Firmware overview

The sonar usually operates in a cycle that consists of the following steps:
  - **Transmitting a pulse:** The sonar generates a pulse using **TIM8** and sends it to the water.
  - **Delay:** The sonar waits for a delay **TIM2** before starting to receive the echo signal if specified.
  - **Receiving the echo signal:** The sonar receives the echo signal using **TIM1** and **ADC4**.
  - **Processing the echo signal:** The sonar processes the echo signal using the DSP module.
  - **Sending the data:** The sonar sends the processed data to the host computer.

As this firmware uses a separated buffer for UART communication, the sonar can be commanded to send the data at any time, not only after a full cycle. It can also start a new processing cycle immediately after finishing the previous one, without needing to wait for the data to be sent.

The code is organized as follows:

1. [**Sonar Module**](/firmware/Core/Src/Sonar)
   - Responsible for the core functionality of the echo sounder
   - Manages the entire echo capture sequence—generating transmit pulses, handling echo reception, running digital signal processing (DSP), and providing real-time updates
   - Centered around the `PingSonar` class in [sonar.cpp](/firmware/Core/Src/Sonar/sonar.cpp)
1. **Board Module**
   - Mainly responsible for managing the sensing aspects of the sonar, reading the supply voltage, PCB temperature, and internal processor temperature
   - Includes some utilities to manage the interface with the hardware interface, mainly focused on the **go to bootloader** logic.
   - Centered around the `SonarBoard` class in [board.cpp](/firmware/Core/Src/Sonar/board.cpp)
1. **Server Module**
   - Responsible for handling the communication between the sonar and the host computer
   - Manages the serial communication, including receiving commands and sending data back to the host
   - Centered around the `PingServer` class in [server.cpp](/firmware/Core/Src/Sonar/server.cpp)
1. [**DSP Module**](/firmware/Core/Src/DSP)
   - Responsible for processing the echo signal received by the sonar
   - Unlike other modules, consists of a series of optimized functions that are used by the `PingSonar` class to process the echo signal
   - Stored by default in the CCM (Core Coupled Memory) of the MCU, to ensure that it can be executed at the highest speed possible

### Configuration

The sonar operational aspects can be configured using the [config.h](/firmware/Core/Inc/config.h) file. This file contains various parameters that can be adjusted to customize the sonar's behavior based on specific requirements. All parameters are documented in the file, making it easy to understand and modify the configuration.

#### Timers

Timers are the core components used to synchronize readings and generate the sonar pulse. This firmware leverages the STM32's interconnect matrix to automate and synchronize the sampling process. Transmission, delay, and reception are all timer-based and interconnected, allowing the firmware to initiate the process and wait for results to be processed using DSP (Digital Signal Processing) algorithms.

The timers utilized are the advanced timers **TIM8** and **TIM1**, and the 32-bit basic timer **TIM2**.

---

##### TIM8 (Advanced)

> Responsible for generating the sonar pulse, initiating the chain of events leading to the echo reception.

**Configuration:**
- **Clock Source:** PLLCLK2 at 144 MHz, providing high precision for pulse generation.
- **Mode:** Configured in **one-pulse mode**, generating a specified number of pulses (determined by the repetition counter) and then stopping.
- **PWM Output:**
  - Channel: CH1
  - Mode: PWM Mode 2
  - Frequency: 115 kHz
  - Duty Cycle: 50%
- **Trigger Event Selection (TRGO):** Configured as **ENABLE**, allowing TIM8 to trigger **TIM2** in sync with the start of the pulse transmission.

---

##### TIM1 (Advanced)

> Controls the timing of each sample acquisition by triggering ADC4 to read the received echo signal.

**Configuration:**
- **Clock Source:** PLLCLK2 at 144 MHz, providing high precision.
- **Mode:** Configured in **one-pulse mode**, generating a fixed number of pulses (determined by the repetition counter) and then stopping, resulting in a fixed number of samples.
- **Trigger Event Selection 2 (TRGO2):** Configured as **Compare Pulse OC1**, used by ADC4 as the source trigger for conversions.

---

##### TIM2

> Acts as a delay generator between the pulse transmission and the start of the echo signal reception, used for the scan start delay.

**Configuration:**
- **Clock Source:** PLLCLK2 at 144 MHz, providing high precision.
- **Mode:** Configured in **one-pulse mode**, generating a delay based on the repetition counter and then stopping.
- **Trigger Event Selection (TRGO):** Configured as **Compare Pulse OC1**, triggering **TIM1** when the delay period ends.

---

#### ADCs

All ADCs use DMA (Direct Memory Access) for efficient data transfer. The configurations for the ADCs are detailed below:

##### ADC1

> ADC1 handles the sensing channels for the sonar system. It operates in continuous mode with 12 bits precision and is configured as follows:

**Configuration:**
- **Clock Source:** The ADC clock is prescaled to **1/12 of the PLL clock**, resulting in a **6 MHz clock** fed to the ADC.
- **Interrupts and DMA:**
  - Both the ADC interrupt and its associated DMA interrupt are disabled (We don't care since we read when we want).
  - The DMA is configured as a **half-word-to-half-word transfer** in circular mode with low priority.
- **Channel Scanning:**
  - The ADC is set to **scan mode** to read multiple channels in sequence.
  - It is **software-triggered** and, once started, continuously reads channels in a loop, storing the results in a dedicated buffer within the firmware's Board module.

**Configured Channels:**
1. **Supply Voltage (5V) Sensing:**
   - **Pin:** PC2
   - **Channel:** ADC1 Channel 8
   - **Rank:** 1
   - **Sampling Time:** 601.5 cycles

2. **PCB Temperature Sensing:**
   - **Pin:** PA2
   - **Channel:** ADC1 Channel 3
   - **Rank:** 2
   - **Sampling Time:** 601.5 cycles

3. **Internal Processor Temperature Sensing:**
   - **Channel:** ADC1 Temperature Sensor channel
   - **Rank:** 3
   - **Sampling Time:** 601.5 cycles

---

##### ADC4

> This one is the main ADC used to receive echo signals from the sonar. It operates in a slave mode with 8 bit precision for fast readings triggered by **TIM1** and is configured as follows:

- **Clock Source:** The ADC clock is fed with PLL clock resulting in a **72 MHz clock**.
- **Interrupts and DMA:**
  - The ADC interrupt is disabled. DMA Interrupt is enabled but its not so important only used as fallback.
  - The DMA is configured as a **half-word-to-byte transfer** in normal mode with low priority.
- **Channel Scanning:**
  - No channel scanning is used. The ADC is configured to read only one channel.
  - It is triggered by **TIM1** and reads the echo signal from the sonar aat fixed intervals by N repetitions set in **TIM1** repetition counter.

**Configured Channels:**
1. **Echo Return Signal:**
   - **Pin:** PB15
   - **Channel:** ADC4 Channel 5
   - **Rank:** 1
   - **Sampling Time:** Dynamically adjusted by firmware, ranging from 7.5 to 61.5 cycles.

---

##### OPAMPs

The operational amplifiers (OPAMPs) have some common configurations:
  - User trimming is enabled.
  - Self-calibration is enabled.

---

- **OPAMP2 and OPAMP3:**
  - Configured in **PGA (Programmable Gain Amplifier)** mode (not connected mode).
  - Default gain is set to a maximum of **16**, but this is dynamically adjusted to optimize signal quality.
  - As these OPAMPs are connected to the external analog sector, all associated pins are set to **analog mode**.

---

- **OPAMP4:**
  - Configured in **standalone mode**.
  - All associated pins are set to **analog mode**.
  - Works in conjunction with the internal **DAC** to adjust the bias (offset) of the received signal since PGA OPAMPs introduces a lot of offset.

##### DAC

> The internal DAC is used to adjust the bias (offset) of the received signal. Ideally the received signal should be centered around the mid range of the ADC, but due to mainly the **PGA** offsets introduced when changing the gain, the signal is not centered. The DAC is used to adjust this offset.

Configuration:
- **OUT1 (PA4)** is configured to be used, it is connected to the **OPAMP4**.
- **Output buffer**: disabled.
- **Trigger Source:** None.

### Dependencies

Before building the firmware, ensure that the necessary dependencies are installed:

1. **Arm GNU Toolchain**
   - The firmware is compiled using `arm-none-eabi-gcc`
   - It can be installed from [Arm Developer](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)

1. **Ping Protocol**
   - The sonar [communicates using the open-source Ping Protocol](#communication-and-interfacing), which enables structured message exchanges between the sonar and external systems
   - For implementation, the sonar utilizes [ping-cpp](https://github.com/bluerobotics/ping-cpp), a C++ library that provides a structured interface for communicating with devices following the Ping Protocol
   - This library facilitates message parsing, serialization, and device interaction

1. **Boost CMake**
   - Boost libraries are required for building the **ping-cpp** submodule
   - You can install it using your package manager or download it from the [Boost website](https://www.boost.org/)

### Building the firmware

The sonar firmware is developed using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) for peripheral configuration, and can be imported into [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) if needed.

However, the primary method of compilation is [CMake](https://cmake.org/), which provides flexibility and ensures compatibility with various development environments.

Follow these steps to build the firmware:

```sh
git submodule update --init --recursive
cd firmware
cmake -B build
cmake --build build --config Release --parallel
```

This process generates the firmware binary, which can be flashed to the STM32 microcontroller.

### Flashing the device

#### Directly, via the command-line

Entering the bootloader mode is essential for reflashing the sonar via the UART1 interface connected to the host computer. To enter bootloader mode, the `BOOT0` pin on the MCU must be pulled high, and the sonar must be reset.

This can be achieved as follows:
1. **Manual Method:**
   - Press the BOOT button on the sonar.
   - Cut and restore power to the sonar while keeping the BOOT button pressed.
   - Release the BOOT button after power is restored.

2. **Automated Method:**
   - Set the `BOOT_CHARGE_PIN` (`PB7`) high.
   - Allow the independent watchdog timer to reset the sonar after a brief delay.

These methods ensure the device transitions into bootloader mode, enabling firmware updates.

If using the UART interface, you can use the **stm32flash** tool to flash the firmware. For example:

```sh
stm32flash -v -g 0x0 -b 115200 -w build/Release/ping-firmware-oss.hex <your device port>
```

It's also possible to flash the firmware via CMake to put the device in **bootloader mode**:

```sh
cmake -B build -DFLASH_DEVICE=/dev/ttyUSB0 && cmake --build build --config Release --parallel --target flash
```

Make sure to have the device in bootloader mode before flashing the firmware and that `<your device port>` is changed to the correct port for the device, by example in a linux environment it could be `/dev/ttyUSB0`.

#### Using Ping Viewer
Firmware options that support the **Automated Method** can also be flashed using the [Ping Viewer](https://docs.bluerobotics.com/ping-viewer/firmware-update/#manual-firmware-update) software, if you prefer a digital interface, and to reduce the tools you need to install. Check more details in the [Firmware Update](https://docs.bluerobotics.com/ping-viewer/firmware-update/) documentation page.


## Project history

For those interested in understanding how this project came to be, and how it has developed over time, here is a brief overview of its development so far:

1. Activation - "sonar is cool", with clear desire in the marine robotics community to be able to play with and test different sonar configurations and algorithms
1. Inspiration - affordable hardware with open communication protocols and established capabilities for basic applications is enticing as a starting point
1. Investigation - Ping Sonar selected as a reverse engineering target, starting from evaluation of chip types and component values
1. Exploration - PCB connections back-traced to create a high level schematic of a viable reference hardware
1. Perspiration - schematic + chip datasheets used to create initial firmware
1. Validation - Ping Protocol implemented for testing, and to ensure validity of reverse-engineering
1. Communication - project licensed and shared openly, to allow the community to explore and contribute
