/*
 * =============================================================================
 *  RECEIVER FIRMWARE — ESP32-S3-Nano Development Board
 *  Framework : ESP-IDF v6.0.1
 *  Protocol  : Bluetooth Low Energy (BLE) using NimBLE stack
 *              — GATT Server (Peripheral role)
 *
 *  Overview:
 *    This device acts as a BLE peripheral. It advertises itself as
 *    "PuppetReceiver" and exposes a single GATT service with three
 *    write-without-response characteristics:
 *
 *      1. SERVO_CHR   — receives pkt_servo_t     → drives two servos via MCPWM
 *      2. AUDIO_CHR   — receives pkt_audio_frag_t / pkt_audio_start_t
 *                        → reassembles audio into a buffer
 *      3. PLAY_CHR    — receives pkt_play_t       → plays back stored audio
 *                        through a speaker amplifier via DAC/PWM
 *
 *  ADC layout (no Wi-Fi = no ADC2 restriction):
 *    ADC1 — microphone input (continuous DMA driver, on the CONTROLLER board)
 *    ADC2 — potentiometers   (oneshot driver, on the CONTROLLER board)
 *    The receiver does not read any ADC channels.
 *
 *  Hardware wiring (adjust GPIO numbers to match your physical board layout):
 *    Servo 1 signal  → GPIO 5   (MCPWM operator 0)
 *    Servo 2 signal  → GPIO 6   (MCPWM operator 1)
 *    Speaker (+)     → GPIO 18  (PWM audio output; add RC low-pass filter)
 *    Speaker (-)     → GND
 *    Servo VCC       → 5 V external rail (NOT the ESP32 3.3 V pin)
 *    Servo GND       → common GND with ESP32
 *
 *  MCPWM servo PWM:
 *    Frequency : 50 Hz  (20 ms period — standard RC servo)
 *    Pulse range: 500 µs (−90°) … 2500 µs (+90°)
 *    The angle-to-pulse formula is linear:
 *      pulse_us = 1500 + angle_deg * (1000 / 90)
 *
 *  Audio playback:
 *    8-bit unsigned PCM samples are output via LEDC PWM at high frequency
 *    (~312.5 kHz carrier). An external RC low-pass filter on GPIO 18 recovers
 *    the analogue waveform. Playback task is created on demand.
 * =============================================================================
 */

/* -------------------------------------------------------------------------- */
/*  Standard & FreeRTOS includes                                              */
/* -------------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* -------------------------------------------------------------------------- */
/*  ESP-IDF system includes                                                   */
/* -------------------------------------------------------------------------- */
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_timer.h"

/* -------------------------------------------------------------------------- */
/*  BLE / NimBLE includes                                                     */
/* -------------------------------------------------------------------------- */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* -------------------------------------------------------------------------- */
/*  MCPWM (servo control) includes                                            */
/* -------------------------------------------------------------------------- */
#include "driver/mcpwm_prelude.h"

/* -------------------------------------------------------------------------- */
/*  LEDC (audio PWM output) includes                                          */
/* -------------------------------------------------------------------------- */
#include "driver/ledc.h"

/* -------------------------------------------------------------------------- */
/*  Logging tag                                                               */
/* -------------------------------------------------------------------------- */
static const char *TAG = "RECV";

/* ==========================================================================
 *  USER CONFIGURATION — edit these to match your hardware
 * ========================================================================== */

/* --- GPIO assignments --- */
#define SERVO1_GPIO         5       /* MCPWM servo 1 signal pin              */
#define SERVO2_GPIO         6       /* MCPWM servo 2 signal pin              */
#define AUDIO_OUT_GPIO      18      /* PWM audio output pin                  */

/* --- MCPWM servo parameters --- */
#define SERVO_TIMEBASE_HZ       1000000     /* 1 MHz timer resolution        */
#define SERVO_TIMEBASE_PERIOD   20000       /* 20 000 µs = 50 Hz             */
#define SERVO_MIN_PULSEWIDTH_US 500         /* Pulse for –90 degrees         */
#define SERVO_MAX_PULSEWIDTH_US 2500        /* Pulse for +90 degrees         */
#define SERVO_MIN_DEGREE        (-90)
#define SERVO_MAX_DEGREE        90

/* --- Audio parameters (must match controller) --- */
#define AUDIO_SAMPLE_RATE_HZ    8000
#define MAX_RECORD_SECONDS      5
#define AUDIO_MAX_SAMPLES       (AUDIO_SAMPLE_RATE_HZ * MAX_RECORD_SECONDS)

#define AUDIO_CHUNK_BYTES       200

/* --- BLE device name (controller scans for this exact name) --- */
#define BLE_DEVICE_NAME         "PuppetReceiver"

/* ==========================================================================
 *  Packet type identifiers  (must be identical in both firmwares)
 * ========================================================================== */
#define PKT_TYPE_SERVO          0x01
#define PKT_TYPE_AUDIO_FRAG     0x02
#define PKT_TYPE_AUDIO_PLAY     0x03
#define PKT_TYPE_AUDIO_START    0x04

/* ==========================================================================
 *  Shared packet structures  (must be identical in both firmwares)
 * ========================================================================== */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;
    int16_t  servo1_angle;
    int16_t  servo2_angle;
} pkt_servo_t;

typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;
    uint16_t frag_index;
    uint16_t total_frags;
    uint16_t payload_len;
    uint8_t  audio[AUDIO_CHUNK_BYTES];
} pkt_audio_frag_t;

typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;
    uint32_t total_samples;
    uint16_t total_frags;
} pkt_audio_start_t;

typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;
} pkt_play_t;

/* ==========================================================================
 *  Custom UUIDs for the Puppet GATT service and characteristics
 *
 *  These 128-bit UUIDs are arbitrary — they just need to match between
 *  controller and receiver. Generated once; never need to change.
 *
 *  Service    : 12345678-1234-1234-1234-1234567890AB
 *  Servo chr  : 12345678-1234-1234-1234-1234567890AC
 *  Audio chr  : 12345678-1234-1234-1234-1234567890AD
 *  Play  chr  : 12345678-1234-1234-1234-1234567890AE
 * ========================================================================== */

/* Service UUID */
static const ble_uuid128_t puppet_svc_uuid =
    BLE_UUID128_INIT(0xAB, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

/* Servo characteristic UUID */
static const ble_uuid128_t servo_chr_uuid =
    BLE_UUID128_INIT(0xAC, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

/* Audio characteristic UUID */
static const ble_uuid128_t audio_chr_uuid =
    BLE_UUID128_INIT(0xAD, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

/* Play characteristic UUID */
static const ble_uuid128_t play_chr_uuid =
    BLE_UUID128_INIT(0xAE, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

/* GATT attribute handles — populated by NimBLE at registration time */
static uint16_t s_servo_chr_handle = 0;
static uint16_t s_audio_chr_handle = 0;
static uint16_t s_play_chr_handle  = 0;

/* ==========================================================================
 *  MCPWM handles
 * ========================================================================== */
static mcpwm_cmpr_handle_t s_servo1_cmp = NULL;  /* Comparator for servo 1  */
static mcpwm_cmpr_handle_t s_servo2_cmp = NULL;  /* Comparator for servo 2  */

/* ==========================================================================
 *  Audio state
 * ========================================================================== */
static uint8_t  *s_audio_buf      = NULL;
static uint32_t  s_audio_samples  = 0;
static uint16_t  s_audio_expected_frags = 0;
static uint16_t  s_audio_rx_frags = 0;
static bool      s_audio_ready    = false;
static SemaphoreHandle_t s_audio_mutex = NULL;

/* BLE connection handle for the currently connected central */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* ==========================================================================
 *  Helper: convert servo angle (degrees) to MCPWM pulse width (microseconds)
 *
 *  Linear interpolation between SERVO_MIN/MAX_PULSEWIDTH_US.
 *  Clamps input to the valid angle range before calculating.
 * ========================================================================== */
static uint32_t angle_to_pulse_us(int16_t angle)
{
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    /* Map [SERVO_MIN_DEGREE … SERVO_MAX_DEGREE] → [MIN_PW … MAX_PW] µs     */
    return (uint32_t)(
        SERVO_MIN_PULSEWIDTH_US +
        ((angle - SERVO_MIN_DEGREE) *
         (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) /
        (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)
    );
}

/* ==========================================================================
 *  MCPWM servo initialisation
 *
 *  Creates one shared timer, two operators (one per servo), two comparators,
 *  and two generators. The generators toggle their output pin based on the
 *  comparator values set by angle_to_pulse_us().
 *
 *  Timer → 2× Operators → 2× Comparators → 2× Generators → GPIO pins
 * ========================================================================== */
static void mcpwm_servo_init(void)
{
    /* ---- Timer ---- */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_HZ,
        .period_ticks  = SERVO_TIMEBASE_PERIOD,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    /* ---- Operator 0 — Servo 1 ---- */
    mcpwm_oper_handle_t oper0 = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper0));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper0, timer));

    /* Comparator 0 — controls servo 1 pulse width */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true, /* Update on timer zero crossing   */
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper0, &cmp_cfg, &s_servo1_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo1_cmp,
                    angle_to_pulse_us(0))); /* Start at centre (0°)          */

    /* Generator 0 — drives SERVO1_GPIO */
    mcpwm_gen_handle_t gen0 = NULL;
    mcpwm_generator_config_t gen_cfg0 = { .gen_gpio_num = SERVO1_GPIO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper0, &gen_cfg0, &gen0));
    /* Set pin HIGH when timer reaches zero (start of period) */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen0,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    /* Set pin LOW when counter reaches comparator value (end of pulse) */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen0,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_servo1_cmp,
                                       MCPWM_GEN_ACTION_LOW)));

    /* ---- Operator 1 — Servo 2 ---- */
    mcpwm_oper_handle_t oper1 = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer));

    /* Comparator 1 — controls servo 2 pulse width */
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &cmp_cfg, &s_servo2_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo2_cmp,
                    angle_to_pulse_us(0)));

    /* Generator 1 — drives SERVO2_GPIO */
    mcpwm_gen_handle_t gen1 = NULL;
    mcpwm_generator_config_t gen_cfg1 = { .gen_gpio_num = SERVO2_GPIO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &gen_cfg1, &gen1));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_servo2_cmp,
                                       MCPWM_GEN_ACTION_LOW)));

    /* ---- Start the timer — both servos begin outputting PWM immediately -- */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM servos initialised — GPIO %d, GPIO %d at 50 Hz",
             SERVO1_GPIO, SERVO2_GPIO);
}

/* ==========================================================================
 *  Audio playback task
 *
 *  Outputs 8-bit PCM samples through a LEDC PWM channel.
 *  The PWM duty cycle (0–255) represents the audio amplitude.
 *  An RC low-pass filter on AUDIO_OUT_GPIO recovers the analogue signal.
 *
 *  LEDC configuration:
 *    Timer frequency : 80 MHz / 256 = 312 500 Hz carrier
 *    Duty resolution : 8-bit (values 0–255 map directly to PCM samples)
 *    Sample rate     : AUDIO_SAMPLE_RATE_HZ (8 kHz)
 *    Between samples : vTaskDelay of 1 000 000 / AUDIO_SAMPLE_RATE_HZ µs
 *
 *  The task self-deletes after playback completes.
 * ========================================================================== */
static void audio_playback_task(void *pvParameters)
{
    xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
    uint32_t total = s_audio_samples;
    xSemaphoreGive(s_audio_mutex);

    ESP_LOGI(TAG, "Playback: %lu audio samples", total);

    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 312500,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = AUDIO_OUT_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 128,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    const uint32_t us_per_sample = 1000000 / AUDIO_SAMPLE_RATE_HZ; /* 125 µs */

    for (uint32_t i = 0; i < total; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, s_audio_buf[i]);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        int64_t t_end = esp_timer_get_time() + us_per_sample;
        while (esp_timer_get_time() < t_end) {}
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ESP_LOGI(TAG, "Playback complete");
    vTaskDelete(NULL);
}

/* ==========================================================================
 *  GATT characteristic access callback
 *
 *  NimBLE calls this function whenever a connected central writes to any of
 *  our three characteristics. The first byte of every write is the pkt_type
 *  field which we use to dispatch to the appropriate handler.
 *
 *  All three characteristics are flagged BLE_GATT_CHR_F_WRITE_NO_RSP so the
 *  central does not need to wait for an acknowledgement — this keeps servo
 *  command latency low.
 * ========================================================================== */
static int gatt_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Only handle WRITE operations; ignore reads on write-only chars */
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    /* Copy mbuf data into a flat local buffer */
    uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t  buf[sizeof(pkt_audio_frag_t)];  /* Largest expected packet     */

    if (data_len > sizeof(buf)) {
        ESP_LOGW(TAG, "Write too large: %u bytes", data_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t pkt_type = buf[0];

    /* ---- Dispatch by packet type ---- */
    if (pkt_type == PKT_TYPE_SERVO && attr_handle == s_servo_chr_handle) {
        /*
         * Servo command — update both MCPWM comparator values immediately.
         * mcpwm_comparator_set_compare_value() is thread-safe.
         */
        if (data_len < sizeof(pkt_servo_t)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        pkt_servo_t *cmd = (pkt_servo_t *)buf;

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
            s_servo1_cmp, angle_to_pulse_us(cmd->servo1_angle)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
            s_servo2_cmp, angle_to_pulse_us(cmd->servo2_angle)));

        ESP_LOGD(TAG, "Servo: ch1=%d° ch2=%d°",
                 cmd->servo1_angle, cmd->servo2_angle);

    } else if (attr_handle == s_audio_chr_handle) {

        if (pkt_type == PKT_TYPE_AUDIO_START) {
            /*
             * Audio transfer start — reset the reassembly buffer.
             * The controller sends this first so we know how many fragments
             * to expect and can verify completeness.
             */
            if (data_len < sizeof(pkt_audio_start_t)) return 0;
            pkt_audio_start_t *start = (pkt_audio_start_t *)buf;

            xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
            s_audio_samples        = (start->total_samples <= AUDIO_MAX_SAMPLES)
                                     ? start->total_samples : AUDIO_MAX_SAMPLES;
            s_audio_expected_frags = start->total_frags;
            s_audio_rx_frags       = 0;
            s_audio_ready          = false;
            xSemaphoreGive(s_audio_mutex);

            ESP_LOGI(TAG, "Audio transfer starting: %lu samples, %u fragments",
                     start->total_samples, start->total_frags);

        } else if (pkt_type == PKT_TYPE_AUDIO_FRAG) {
            /*
             * Audio fragment — copy payload bytes to the correct offset in
             * the reassembly buffer.
             *
             * frag_index × AUDIO_CHUNK_BYTES gives the byte offset.
             * We bounds-check before writing to prevent buffer overflow.
             */
            if (data_len < (sizeof(pkt_audio_frag_t) - AUDIO_CHUNK_BYTES + 1)) return 0;
            pkt_audio_frag_t *frag = (pkt_audio_frag_t *)buf;

            uint32_t offset = (uint32_t)frag->frag_index * AUDIO_CHUNK_BYTES;

            if (offset + frag->payload_len <= AUDIO_MAX_SAMPLES && s_audio_buf) {
                memcpy(s_audio_buf + offset, frag->audio, frag->payload_len);

                xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                s_audio_rx_frags++;
                if (s_audio_rx_frags >= s_audio_expected_frags) {
                    s_audio_ready = true;
                    ESP_LOGI(TAG, "Audio transfer complete — %lu samples ready",
                             s_audio_samples);
                }
                xSemaphoreGive(s_audio_mutex);

                ESP_LOGD(TAG, "Audio frag %u/%u received",
                         frag->frag_index + 1, frag->total_frags);
            } else {
                ESP_LOGW(TAG, "Audio frag offset out of range: %lu", offset);
            }

        }

    } else if (pkt_type == PKT_TYPE_AUDIO_PLAY && attr_handle == s_play_chr_handle) {
        /*
         * Play command — start audio playback if a complete recording is
         * available. We create the playback task on Core 1 to avoid
         * interfering with BLE processing on Core 0.
         */
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        bool ready = s_audio_ready && s_audio_samples > 0;
        xSemaphoreGive(s_audio_mutex);

        if (ready) {
            ESP_LOGI(TAG, "Play command received — starting playback");
            xTaskCreatePinnedToCore(audio_playback_task, "audio_play",
                                    4096, NULL, 5, NULL, 1);
        } else {
            ESP_LOGW(TAG, "Play command received but no audio is ready");
        }
    }

    return 0;
}

/* ==========================================================================
 *  GATT service definition table
 *
 *  NimBLE reads this static table at startup to register our service and
 *  characteristics. The table MUST be terminated with a zero-filled entry.
 *
 *  All three characteristics use BLE_GATT_CHR_F_WRITE_NO_RSP (Write Command)
 *  so the central can fire-and-forget at high frequency without waiting for
 *  ATT acknowledgements. This is critical for servo latency.
 * ========================================================================== */
static const struct ble_gatt_svc_def s_puppet_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &puppet_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {

            /* ---- Servo characteristic ---- */
            {
                .uuid       = &servo_chr_uuid.u,
                .access_cb  = gatt_write_cb,
                .val_handle = &s_servo_chr_handle,
                .flags      = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },

            /* ---- Audio characteristic ---- */
            {
                .uuid       = &audio_chr_uuid.u,
                .access_cb  = gatt_write_cb,
                .val_handle = &s_audio_chr_handle,
                .flags      = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },

            /* ---- Play characteristic ---- */
            {
                .uuid       = &play_chr_uuid.u,
                .access_cb  = gatt_write_cb,
                .val_handle = &s_play_chr_handle,
                .flags      = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },

            { 0 }, /* Terminate characteristics array */
        },
    },
    { 0 }, /* Terminate services array */
};

/* ==========================================================================
 *  GATT server initialisation
 * ========================================================================== */
static void gatt_server_init(void)
{
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_puppet_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(s_puppet_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "GATT server registered — 3 characteristics ready");
}

/* ==========================================================================
 *  BLE advertising
 *
 *  Advertises the device name "PuppetReceiver" and the puppet service UUID
 *  so the controller can find it by name during scanning.
 * ========================================================================== */
static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields  fields     = {0};
    int rc;

    /* Include full device name in advertising payload */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name        = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len    = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    /* Connectable, undirected advertising at ~100 ms interval */
    adv_params.conn_mode  = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode  = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min   = BLE_GAP_ADV_ITVL_MS(100);
    adv_params.itvl_max   = BLE_GAP_ADV_ITVL_MS(150);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising as \"%s\"...", BLE_DEVICE_NAME);
}

/* ==========================================================================
 *  NimBLE host sync callback — called when BLE stack is ready
 * ========================================================================== */
static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0); /* Use public address               */
    assert(rc == 0);

    uint8_t addr[6];
    ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL);
    ESP_LOGI(TAG, "BLE ready. Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    ble_advertise();
}

/* ==========================================================================
 *  NimBLE host reset callback — called when BLE stack resets unexpectedly
 * ========================================================================== */
static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE stack reset; reason=%d — will re-advertise", reason);
}

/* ==========================================================================
 *  NimBLE host task — runs the NimBLE event loop
 * ========================================================================== */
static void nimble_host_task(void *pvParameters)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run(); /* Blocks here until nimble_port_stop() is called     */
    nimble_port_freertos_deinit();
}

/* ==========================================================================
 *  app_main
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "  ESP32-S3 Puppet Receiver");
    ESP_LOGI(TAG, "  ESP-IDF v6.0.1  |  BLE NimBLE");
    ESP_LOGI(TAG, "================================");

    /* --- NVS flash (required by BLE stack) --- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* --- Audio reassembly buffer (prefer PSRAM) --- */
    s_audio_buf = (uint8_t *)heap_caps_malloc(
        AUDIO_MAX_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_audio_buf) s_audio_buf = (uint8_t *)malloc(AUDIO_MAX_SAMPLES);
    if (!s_audio_buf) {
        ESP_LOGE(TAG, "FATAL: Cannot allocate %d byte audio buffer", AUDIO_MAX_SAMPLES);
        return;
    }

    s_audio_mutex = xSemaphoreCreateMutex();

    /* --- Servo hardware --- */
    mcpwm_servo_init();

    /* --- BLE stack --- */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    /* Set preferred MTU — must match or exceed the controller's request (512).
     * Audio fragments are 207 bytes; without this the ATT default (23 bytes)
     * would truncate every fragment, producing noise during playback.        */
    ble_att_set_preferred_mtu(512);

    /* Set device name before sync so it appears in advertising */
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    /* Register GATT service */
    gatt_server_init();

    /* Register NimBLE callbacks */
    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    /* Start the NimBLE host task on Core 0 */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "Receiver initialised. Waiting for controller connection...");
    ESP_LOGI(TAG, "  Servo 1 → GPIO %d", SERVO1_GPIO);
    ESP_LOGI(TAG, "  Servo 2 → GPIO %d", SERVO2_GPIO);
    ESP_LOGI(TAG, "  Audio   → GPIO %d", AUDIO_OUT_GPIO);
}