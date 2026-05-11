# ==============================================================================
#  CMakeLists.txt — COMPONENT (Receiver/main)
#  Registers the main component with all required IDF component dependencies.
#
#  Required components:
#    nvs_flash  — Non-volatile storage (BLE bonding keys, boot config)
#    bt         — Bluetooth stack (NimBLE GATT server, GAP peripheral)
#    driver     — GPIO + MCPWM (servo control) + LEDC (audio PWM playback)
#    freertos   — FreeRTOS kernel (tasks, queues, semaphores)
#    esp_timer  — High-resolution timer (used internally by NimBLE)
#    heap       — Heap capabilities API (PSRAM allocation for audio buffer)
#    esp_adc    — Not used on receiver but included for consistency; remove
#                 if binary size is a concern
# ==============================================================================

idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        bt
        driver
        esp_driver_mcpwm
        esp_driver_ledc      #(for audio PWM playback)
        freertos
        esp_timer
        heap
)