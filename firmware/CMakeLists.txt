cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ping-firmware-oss)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add ping-cpp lib
set(PING_CPP_BUILD_DIR ${CMAKE_BINARY_DIR}/ping-cpp)
add_custom_target(build-ping-cpp ALL
    COMMAND ${CMAKE_COMMAND} -B${PING_CPP_BUILD_DIR} -S${CMAKE_SOURCE_DIR}/Lib/ping-cpp -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    COMMAND ${CMAKE_COMMAND} --build ${PING_CPP_BUILD_DIR} --config ${CMAKE_BUILD_TYPE}
    COMMENT "Building ping-cpp with the host compiler"
)
add_dependencies(${CMAKE_PROJECT_NAME} build-ping-cpp)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    # DSP
    "Core/Src/DSP/echo_finder.c"
    "Core/Src/DSP/steady_state_finder.c"
    "Core/Src/DSP/u8_compress_profile.c"
    "Core/Src/DSP/u8_fast_abs_delta.c"
    "Core/Src/DSP/u8_fast_max.c"
    "Core/Src/DSP/u8_fast_mean.c"
    "Core/Src/DSP/u8_fast_normalize.c"
    "Core/Src/DSP/u8_fast_half_square.c"
    "Core/Src/DSP/u8_fast_std_dev.c"
    "Core/Src/DSP/u8_fast_threshold_cut.c"

    # Sonar
    "Core/Src/Sonar/board.cpp"
    "Core/Src/Sonar/sonar.cpp"
    "Core/Src/Sonar/server.cpp"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    "Lib/ping-cpp/src/message" # Generated headers from ping-cpp
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Create binary and hex files
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Creating binary and hex file"
)
