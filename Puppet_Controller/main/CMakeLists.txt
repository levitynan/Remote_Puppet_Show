# ==============================================================================
#  CMakeLists.txt — COMPONENT (Controller/main)
#  ESP-IDF v6.0.1 | ESP32-S3
#
#  Source files:
#    main.c   — Application firmware (BLE central, ADC, servo commands, audio)
#    peer.c   — NimBLE peer tracking and GATT service discovery helper.
#               Sourced from ESP-IDF examples/bluetooth/nimble/common/
#               nimble_central_utils/peer.c — must be compiled as part of this
#               component because nimble_central_utils is an example utility,
#               not a redistributable IDF library component.
#    misc.c   — NimBLE BLE utility helpers (print_bytes, addr_str, etc.)
#               Sourced from the same nimble_central_utils directory.
#
#  Required IDF components:
#    nvs_flash        — NVS (NimBLE bonding key storage, boot config)
#    bt               — NimBLE BLE stack (GAP central, GATT client)
#    esp_adc          — ADC oneshot (potentiometers) + continuous (microphone)
#    driver           — Peripheral driver umbrella (GPIO, MCPWM, LEDC, etc.)
#    esp_driver_gpio  — GPIO driver (IDF v5+ split from driver umbrella)
#    freertos         — FreeRTOS kernel (tasks, semaphores, event groups)
#    esp_timer        — High-resolution timer (used internally by NimBLE)
#    heap             — Heap capabilities API (PSRAM audio buffer allocation)
# ==============================================================================

idf_component_register(
    SRCS
        "main.c"
        "peer.c"
        "misc.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        bt
        esp_adc
        driver
        esp_driver_gpio
        freertos
        esp_timer
        heap
)