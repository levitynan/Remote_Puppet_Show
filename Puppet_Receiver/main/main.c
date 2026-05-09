/*
 * =============================================================================
 *  RECEIVER FIRMWARE — ESP32-S3-Nano Development Board
 *  Framework : ESP-IDF v6.0.1
 *  Purpose   : Receive servo angle commands from the controller board via
 *              ESP-NOW, then drive two RC servo motors using the MCPWM
 *              (Motor Control PWM) peripheral.
 *
 *  Hardware wiring (adjust GPIO numbers to match your board layout):
 *    Servo 1 signal  → GPIO 5
 *    Servo 2 signal  → GPIO 6
 *    Both servo VCC  → External 5 V rail (DO NOT power from ESP32 3.3 V pin)
 *    Both servo GND  → Common GND with ESP32
 *
 *  ESP-NOW notes:
 *    • No pairing / provisioning is needed on the receiver side.
 *    • The receiver accepts packets from ANY MAC address (promiscuous peer).
 *    • The packet payload is a shared struct (servo_command_t) that must be
 *      identical on both the controller and this firmware.
 * =============================================================================
 */

/* -------------------------------------------------------------------------- */
/*  Standard & FreeRTOS includes                                              */
/* -------------------------------------------------------------------------- */
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* -------------------------------------------------------------------------- */
/*  ESP-IDF driver includes                                                    */
/* -------------------------------------------------------------------------- */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "nvs_flash.h"

/* MCPWM (new "prelude" API introduced in IDF v5, used in v6 as well) */
#include "driver/mcpwm_prelude.h"

/* -------------------------------------------------------------------------- */
/*  Logging tag                                                               */
/* -------------------------------------------------------------------------- */
static const char *TAG = "RECEIVER";

/* -------------------------------------------------------------------------- */
/*  Servo PWM timing constants                                                 */
/*                                                                             */
/*  Most hobby RC servos interpret a 50 Hz PWM signal where:                  */
/*    • 500  µs pulse → –90° (full counter-clockwise)                         */
/*    • 1500 µs pulse →   0° (centre / neutral)                               */
/*    • 2500 µs pulse → +90° (full clockwise)                                 */
/*  Adjust SERVO_MIN_PULSEWIDTH_US / SERVO_MAX_PULSEWIDTH_US if your servo    */
/*  uses a different range (e.g. 1000–2000 µs for a 180° servo).             */
/* -------------------------------------------------------------------------- */
#define SERVO_TIMEBASE_RESOLUTION_HZ  1000000  /* Timer resolution: 1 MHz → 1 tick = 1 µs     */
#define SERVO_TIMEBASE_PERIOD         20000    /* 20 000 ticks = 20 ms → 50 Hz PWM frequency  */

#define SERVO_MIN_PULSEWIDTH_US       500      /* Pulse width for minimum angle (–90°)         */
#define SERVO_MAX_PULSEWIDTH_US       2500     /* Pulse width for maximum angle (+90°)         */
#define SERVO_MIN_DEGREE              (-90)    /* Minimum angle in degrees                     */
#define SERVO_MAX_DEGREE              90       /* Maximum angle in degrees                     */

/* GPIO pins connected to the servo signal wires */
#define SERVO1_GPIO                   5        /* Signal wire of servo motor 1                 */
#define SERVO2_GPIO                   6        /* Signal wire of servo motor 2                 */

/* -------------------------------------------------------------------------- */
/*  Shared command structure                                                   */
/*  *** This struct MUST be byte-for-byte identical in controller/main.c ***  */
/* -------------------------------------------------------------------------- */
typedef struct {
    int16_t servo1_angle;   /* Target angle for servo 1, range –90 to +90 degrees */
    int16_t servo2_angle;   /* Target angle for servo 2, range –90 to +90 degrees */
} servo_command_t;

/* -------------------------------------------------------------------------- */
/*  Module-level handles for the two MCPWM comparators                        */
/*  These are set during initialisation and used by the ESP-NOW callback.     */
/* -------------------------------------------------------------------------- */
static mcpwm_cmpr_handle_t s_servo1_comparator = NULL;
static mcpwm_cmpr_handle_t s_servo2_comparator = NULL;

/* -------------------------------------------------------------------------- */
/*  FreeRTOS queue used to pass commands from the ISR-context ESP-NOW         */
/*  callback into the main task context where MCPWM writes are safe.          */
/* -------------------------------------------------------------------------- */
static QueueHandle_t s_servo_cmd_queue = NULL;

/* ========================================================================== */
/*  Helper: angle → compare value (pulse width in µs)                         */
/*                                                                             */
/*  Linear interpolation between SERVO_MIN_PULSEWIDTH_US and                  */
/*  SERVO_MAX_PULSEWIDTH_US across the full degree range.                     */
/*                                                                             */
/*  Example: angle =  0  → 1500 µs (centre)                                  */
/*           angle = 90  → 2500 µs (full CW)                                  */
/*           angle = -90 → 500  µs (full CCW)                                 */
/* ========================================================================== */
static inline uint32_t angle_to_compare(int angle)
{
    /* Clamp input to the valid degree range to protect the servo hardware */
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    return (uint32_t)(
        (angle - SERVO_MIN_DEGREE) *
        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
        (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)
        + SERVO_MIN_PULSEWIDTH_US
    );
}

/* ========================================================================== */
/*  MCPWM Initialisation                                                       */
/*                                                                             */
/*  The new IDF v5/v6 MCPWM API uses an object model:                         */
/*    Timer  → defines the PWM period (50 Hz here).                           */
/*    Operator → connects to the timer; owns comparators and generators.       */
/*    Comparator → holds the compare value that determines pulse width.        */
/*    Generator → maps comparator events to a GPIO pin.                        */
/*                                                                             */
/*  Both servos share one MCPWM group (group_id = 0) and one timer so that    */
/*  their PWM signals are perfectly synchronised.                              */
/* ========================================================================== */
static void mcpwm_servo_init(void)
{
    ESP_LOGI(TAG, "Initialising MCPWM timer (50 Hz, 1 µs resolution)");

    /* ------------------------------------------------------------------ */
    /*  Timer — shared by both servo operators                            */
    /* ------------------------------------------------------------------ */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id       = 0,                           /* MCPWM group 0 (ESP32-S3 has 2 groups) */
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT, /* Use the default clock source          */
        .resolution_hz  = SERVO_TIMEBASE_RESOLUTION_HZ,/* 1 MHz → each tick = 1 µs              */
        .period_ticks   = SERVO_TIMEBASE_PERIOD,        /* 20 000 ticks = 20 ms = 50 Hz          */
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,    /* Count up, reset on period overflow    */
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    /* ------------------------------------------------------------------ */
    /*  Operator 1 — controls Servo 1                                     */
    /* ------------------------------------------------------------------ */
    mcpwm_oper_handle_t oper1 = NULL;
    mcpwm_operator_config_t operator_config1 = {
        .group_id = 0,  /* Must be in the same group as the timer */
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config1, &oper1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer)); /* Bind operator1 to the shared timer */

    /* Comparator 1 — holds the pulse width value for servo 1 */
    mcpwm_comparator_config_t comparator_config1 = {
        .flags.update_cmp_on_tez = true, /* Update compare value at timer-zero event (glitch-free) */
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config1, &s_servo1_comparator));

    /* Generator 1 — drives the physical GPIO pin for servo 1 */
    mcpwm_gen_handle_t gen1 = NULL;
    mcpwm_generator_config_t gen1_config = {
        .gen_gpio_num = SERVO1_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &gen1_config, &gen1));

    /* Set initial compare value → servo 1 goes to centre (0°) */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo1_comparator, angle_to_compare(0)));

    /* Generator 1 action: go HIGH at timer-zero, go LOW at comparator match */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_servo1_comparator,
                                       MCPWM_GEN_ACTION_LOW)));

    /* ------------------------------------------------------------------ */
    /*  Operator 2 — controls Servo 2                                     */
    /* ------------------------------------------------------------------ */
    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0,  /* Same MCPWM group */
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer)); /* Both operators share the same timer */

    /* Comparator 2 — holds the pulse width value for servo 2 */
    mcpwm_comparator_config_t comparator_config2 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &s_servo2_comparator));

    /* Generator 2 — drives the physical GPIO pin for servo 2 */
    mcpwm_gen_handle_t gen2 = NULL;
    mcpwm_generator_config_t gen2_config = {
        .gen_gpio_num = SERVO2_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &gen2_config, &gen2));

    /* Set initial compare value → servo 2 goes to centre (0°) */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo2_comparator, angle_to_compare(0)));

    /* Generator 2 actions: identical logic to Generator 1 */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_servo2_comparator,
                                       MCPWM_GEN_ACTION_LOW)));

    /* ------------------------------------------------------------------ */
    /*  Enable and start the shared timer                                  */
    /*  MCPWM_TIMER_START_NO_STOP → runs indefinitely until explicitly     */
    /*  stopped with mcpwm_timer_start_stop(..., MCPWM_TIMER_STOP_EMPTY). */
    /* ------------------------------------------------------------------ */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM initialised — Servo1 GPIO %d, Servo2 GPIO %d", SERVO1_GPIO, SERVO2_GPIO);
}

/* ========================================================================== */
/*  ESP-NOW receive callback                                                   */
/*                                                                             */
/*  This function is called from the Wi-Fi task context (NOT your app task).  */
/*  Avoid doing heavy work here. Instead, copy the data into a FreeRTOS queue */
/*  and process it from a dedicated task.                                      */
/* ========================================================================== */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len)
{
    /* Validate payload length — reject malformed packets */
    if (len != sizeof(servo_command_t)) {
        ESP_LOGW(TAG, "Received packet of unexpected size %d (expected %d) — ignoring",
                 len, (int)sizeof(servo_command_t));
        return;
    }

    /* Copy the received bytes into a local struct for type safety */
    servo_command_t cmd;
    memcpy(&cmd, data, sizeof(servo_command_t));

    ESP_LOGI(TAG, "ESP-NOW RX from " MACSTR " — servo1=%d°, servo2=%d°",
             MAC2STR(recv_info->src_addr),
             cmd.servo1_angle,
             cmd.servo2_angle);

    /* Send to the servo control task via FreeRTOS queue.                  */
    /* xQueueSendFromISR is safe here because this callback runs in the    */
    /* Wi-Fi task context. Use portMAX_DELAY would block; 0 means discard  */
    /* if the queue is full (safe for real-time control).                  */
    BaseType_t higher_prio_woken = pdFALSE;
    xQueueSendFromISR(s_servo_cmd_queue, &cmd, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

/* ========================================================================== */
/*  Wi-Fi + ESP-NOW Initialisation                                             */
/*                                                                             */
/*  ESP-NOW requires Wi-Fi to be started in station mode but does NOT         */
/*  require an actual AP connection. No SSID or password is needed.           */
/* ========================================================================== */
static void wifi_espnow_init(void)
{
    /* ------------------------------------------------------------------ */
    /*  NVS flash — required by the Wi-Fi driver                          */
    /* ------------------------------------------------------------------ */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated; erase and re-init */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ------------------------------------------------------------------ */
    /*  Networking stack                                                   */
    /* ------------------------------------------------------------------ */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ------------------------------------------------------------------ */
    /*  Wi-Fi driver — station mode, no connection required               */
    /* ------------------------------------------------------------------ */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Print this board's MAC address so it can be pasted into the controller */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "Receiver MAC address: " MACSTR, MAC2STR(mac));
    ESP_LOGI(TAG, ">>> Paste the above MAC into RECEIVER_MAC_ADDR in controller/main/main.c <<<");

    /* ------------------------------------------------------------------ */
    /*  ESP-NOW                                                            */
    /* ------------------------------------------------------------------ */
    ESP_ERROR_CHECK(esp_now_init());

    /* Register the receive callback — called whenever a packet arrives */
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW initialised and listening for servo commands");
}

/* ========================================================================== */
/*  Servo control task                                                         */
/*                                                                             */
/*  Blocks on the queue. Whenever a new command arrives, it updates both      */
/*  MCPWM comparator values, causing the servo signals to change on the very  */
/*  next PWM period (~20 ms maximum latency).                                 */
/* ========================================================================== */
static void servo_control_task(void *pvParameters)
{
    servo_command_t cmd;

    ESP_LOGI(TAG, "Servo control task started — waiting for commands...");

    while (1) {
        /* Block indefinitely until a command is placed in the queue */
        if (xQueueReceive(s_servo_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {

            ESP_LOGI(TAG, "Moving servo1 → %d°, servo2 → %d°",
                     cmd.servo1_angle, cmd.servo2_angle);

            /* Update servo 1 — converts angle to µs and writes to MCPWM comparator */
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
                s_servo1_comparator, angle_to_compare(cmd.servo1_angle)));

            /* Update servo 2 */
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
                s_servo2_comparator, angle_to_compare(cmd.servo2_angle)));
        }
    }

    /* Should never reach here, but included for completeness */
    vTaskDelete(NULL);
}

/* ========================================================================== */
/*  app_main — entry point                                                     */
/* ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "  ESP32-S3 Servo Receiver");
    ESP_LOGI(TAG, "  ESP-IDF v6.0.1");
    ESP_LOGI(TAG, "==============================");

    /* Create a queue that holds up to 5 servo commands.
     * Depth of 5 provides a small buffer without accumulating stale commands. */
    s_servo_cmd_queue = xQueueCreate(5, sizeof(servo_command_t));
    if (s_servo_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create servo command queue — halting");
        return;
    }

    /* Initialise MCPWM (servos go to 0° / centre on startup) */
    mcpwm_servo_init();

    /* Initialise Wi-Fi and ESP-NOW */
    wifi_espnow_init();

    /* Start the servo control task on core 1, priority 5 */
    xTaskCreatePinnedToCore(
        servo_control_task,     /* Task function                   */
        "servo_ctrl",           /* Task name (for debugging)       */
        4096,                   /* Stack size in bytes             */
        NULL,                   /* Task parameter (not used)       */
        5,                      /* Priority (higher = more urgent) */
        NULL,                   /* Task handle (not stored)        */
        1                       /* CPU core (1 = second core)      */
    );

    ESP_LOGI(TAG, "Initialisation complete. Receiver is active.");
}

/*
 * =============================================================================
 * 
 * Project Architecture
 *  independent ESP-IDF v6.0.1 projects communicate over ESP-NOW (a connectionless Wi-Fi protocol from Espressif — no router, no pairing, sub-millisecond overhead). The receiver translates incoming angle commands into PWM signals using the MCPWM peripheral.
 * 
 * Receiver (receiver/main/main.c)
 * The receiver uses the new MCPWM "prelude" API (introduced in IDF v5, retained in v6) which follows an object model: Timer → Operator → Comparator → Generator.
 * 
 * One shared timer runs at 1 MHz resolution, 20 ms period (50 Hz) — the standard RC servo frequency
 * 
 * Two separate operators (one per servo) connect to that single timer, so both PWM signals are phase-aligned
 * 
 * Comparators hold the pulse-width value in microseconds (500–2500 µs for –90° to +90°)
 * 
 * Generators map comparator events to the physical GPIO pins (GPIO 5 and GPIO 6 — change to suit your board)

 * The ESP-NOW receive callback runs in the Wi-Fi task context, so it only enqueues into a FreeRTOS queue — a dedicated servo_control_task handles the actual MCPWM writes safely

 * Controller (controller/main/main.c)
 * Reads two potentiometers via the IDF v5+ adc_oneshot driver with curve-fitting calibration for accurate mV readings
 * 
 * Maps 0–3300 mV linearly to –90° to +90° via mv_to_angle()

 * Sends a servo_command_t struct to the receiver at 20 Hz via esp_now_send()

 * The espnow_send_cb() callback logs delivery success/failure for each packet

 * Shared servo_command_t Struct
 * c
 * typedef struct {
 *     int16_t servo1_angle;   // –90 to +90 degrees
 *     int16_t servo2_angle;   // –90 to +90 degrees
 * } servo_command_t;
 * This struct must be byte-for-byte identical in both firmwares.
 * 
 * Setup Steps
 * Flash the receiver first. Open its serial monitor and copy the MAC address it prints (e.g., AA:BB:CC:DD:EE:FF).

 * Paste that MAC into RECEIVER_MAC_ADDR in controller/main/main.c.

 * Flash the controller. It will immediately begin sending angle commands.

 * Wire servos to GPIO 5 and GPIO 6 on the receiver board. Power servo VCC from a 5 V external rail — never from the ESP32's 3.3 V pin.
 * 
 * If you don't have potentiometers, replace the adc_read_mv() calls in the controller task with any other angle source (e.g., hardcoded test values, UART input, or a joystick).
 * 
 * Key Configuration Constants to Adjust
 * Constant	File	Purpose
 * SERVO1_GPIO / SERVO2_GPIO	receiver	GPIO pins for servo signal wires
 * SERVO_MIN/MAX_PULSEWIDTH_US	receiver	Tune to your servo's actual pulse range
 * POT1/2_ADC_CHANNEL	controller	ADC channels for your potentiometer GPIOs
 * SEND_INTERVAL_MS	controller	Command update rate (default 50 ms = 20 Hz)
 * RECEIVER_MAC_ADDR	controller	Must match your receiver board's MAC
 * Prepared using Claude Sonnet 4.6 Thinking
 * Follow-ups

 * Complete ESP-IDF code for ESP32-S3 ESP-NOW servo receiver — ready to compile & flash
 * Computer
 * ​ * 
 * 
 * ESP-IDF project setup files for dual ESP32-S3 servo control in VS Code
 * Computer
 * 
 * ESP32-S3-Nano default GPIO pins for MCPWM servo control
 * 
 * ESP-NOW bidirectional communication example for ESP32-S3
 * 
 * Power requirements for two servos on ESP32-S3 receiver
 * =============================================================================
 */