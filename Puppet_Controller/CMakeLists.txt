# ==============================================================================
#  CMakeLists.txt — TOP LEVEL (Controller)
#  Project  : Puppet BLE Controller
#  Target   : ESP32-S3
#  IDF Ver  : v6.0.1
#
#  This is the top-level CMake entry point required by ESP-IDF. It must be
#  placed in the root of the project directory (alongside sdkconfig.defaults).
#  The actual component source is declared in main/CMakeLists.txt.
# ==============================================================================

cmake_minimum_required(VERSION 3.16)

# Include the ESP-IDF project helper — sets up toolchain, component discovery,
# flash targets, etc. IDF_PATH must be set in the environment (done
# automatically by the ESP-IDF VSCode extension or by sourcing export.sh).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project name — used as the output binary name (controller.bin / controller.elf)
project(controller)