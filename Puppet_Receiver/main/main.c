/*
 * =============================================================================
 *  RECEIVER FIRMWARE — ESP32-S3-Nano Development Board
 *  Framework : ESP-IDF v6.0.1
 *
 *  Features:
 *    1. Receive servo angle commands → drive two RC servo motors via MCPWM.
 *    2. Receive fragmented audio packets from the controller → reassemble into
 *       a PCM buffer.
 *    3. On receipt of a PKT_TYPE_AUDIO_PLAY command → play the stored audio
 *       through an analogue Audio Amplifier Module via I2S (standard TX mode).
 *
 *  Audio output path (ESP32-S3 has no built-in DAC):
 *    The ESP32-S3 I2S peripheral is configured in standard TX mode, outputting
 *    16-bit PCM samples at 8 kHz to a single GPIO (I2S data out).  
 *    Connect I2S_DOUT_GPIO → amplifier module input.
 *    Connect I2S_BCLK_GPIO + I2S_LRCLK_GPIO if using a digital-input amp
 *    (e.g. MAX98357A). If your amp module expects an analogue signal, add a
 *    simple RC low-pass filter (10 kΩ + 10 nF) after I2S_DOUT_GPIO to recover
 *    the analogue audio from the PWM-like digital output.
 *
 *  Hardware wiring (adjust GPIO numbers to match your physical board):
 *    Servo 1 signal  → GPIO 5
 *    Servo 2 signal  → GPIO 6
 *    I2S BCLK        → GPIO 15   (bit clock to amp module)
 *    I2S LRCLK/WS    → GPIO 16   (left-right / word-select clock)
 *    I2S DOUT        → GPIO 17   (serial audio data → amp module DIN)
 *    Amp VCC         → 3.3 V or 5 V (check your module's datasheet)
 *    Amp GND         → GND
 * =============================================================================
 */

/* -------------------------------------------------------------------------- */
/*  Standard & FreeRTOS includes                                              */
/* -------------------------------------------------------------------------- */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* -------------------------------------------------------------------------- */
/*  ESP-IDF driver & system includes                                          */
/* -------------------------------------------------------------------------- */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "nvs_flash.h"

/* MCPWM — servo motor control */
#include "driver/mcpwm_prelude.h"

/* I2S — audio output (new IDF v5/v6 "std" driver) */
#include "driver/i2s_std.h"
#include "driver/i2s_types.h"

/* -------------------------------------------------------------------------- */
/*  Logging tag                                                               */
/* -------------------------------------------------------------------------- */
static const char *TAG = "RECV";

/* ==========================================================================
 *  USER CONFIGURATION — edit these to match your hardware
 * ========================================================================== */

/* --- Servo GPIO pins --- */
#define SERVO1_GPIO             5
#define SERVO2_GPIO             6

/* --- Audio I2S GPIO pins ---
 * For a MAX98357A or similar I2S digital-input amplifier module.
 * If using an analogue amp module, connect only DOUT through an RC filter.  */
#define I2S_BCLK_GPIO           15   /* Bit clock                            */
#define I2S_LRCLK_GPIO          16   /* Word select (left/right clock)       */
#define I2S_DOUT_GPIO           17   /* Serial data out → amp DIN            */

/* ==========================================================================
 *  Servo PWM timing constants
 * ========================================================================== */
#define SERVO_TIMEBASE_RESOLUTION_HZ  1000000  /* 1 MHz → 1 tick = 1 µs     */
#define SERVO_TIMEBASE_PERIOD         20000    /* 20 000 ticks = 20 ms = 50 Hz */
#define SERVO_MIN_PULSEWIDTH_US       500
#define SERVO_MAX_PULSEWIDTH_US       2500
#define SERVO_MIN_DEGREE              (-90)
#define SERVO_MAX_DEGREE              90

/* ==========================================================================
 *  Audio constants
 * ========================================================================== */
#define AUDIO_SAMPLE_RATE_HZ    8000    /* Must match controller             */
#define AUDIO_BITS_PER_SAMPLE   16      /* I2S TX uses 16-bit frames         */

/* Maximum audio buffer — same as controller's AUDIO_MAX_SAMPLES            */
#define AUDIO_MAX_SAMPLES       (AUDIO_SAMPLE_RATE_HZ * 5)   /* 5 s = 40000 */

/* Audio chunk bytes per fragment — must match controller                    */
#define AUDIO_CHUNK_BYTES       200

/* ==========================================================================
 *  Packet type identifiers — MUST match controller values
 * ========================================================================== */
#define PKT_TYPE_SERVO          0x01
#define PKT_TYPE_AUDIO_FRAG     0x02
#define PKT_TYPE_AUDIO_PLAY     0x03
#define PKT_TYPE_AUDIO_START    0x04

/* ==========================================================================
 *  Packet structures — MUST be identical to controller
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
 *  Module-level handles and state
 * ========================================================================== */

/* MCPWM comparator handles for the two servo motors */
static mcpwm_cmpr_handle_t s_servo1_cmp = NULL;
static mcpwm_cmpr_handle_t s_servo2_cmp = NULL;

/* I2S channel handle for audio output */
static i2s_chan_handle_t s_i2s_tx_handle = NULL;

/* Audio receive buffer */
static uint8_t  *s_audio_buf       = NULL;  /* 8-bit PCM storage            */
static uint32_t  s_audio_samples   = 0;     /* Number of valid samples stored */
static uint16_t  s_expected_frags  = 0;     /* Fragments expected for current transfer */
static uint16_t  s_received_frags  = 0;     /* Fragments received so far    */

/* Mutex protecting audio buffer and counters */
static SemaphoreHandle_t s_audio_mutex = NULL;

/* Event group bit used to signal the playback task */
#define PLAY_EVENT_BIT  BIT0
static EventGroupHandle_t s_play_event_group = NULL;

/* FreeRTOS queue: servo commands from ESP-NOW callback → servo update task */
static QueueHandle_t s_servo_cmd_queue = NULL;

/* ==========================================================================
 *  Helper: angle (degrees) → MCPWM comparator value (µs pulse width)
 * ========================================================================== */
static inline uint32_t angle_to_compare(int angle)
{
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    return (uint32_t)(
        (angle - SERVO_MIN_DEGREE) *
        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
        (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)
        + SERVO_MIN_PULSEWIDTH_US
    );
}

/* ==========================================================================
 *  MCPWM servo initialisation
 *
 *  One shared timer drives two operators (one per servo).
 *  Generator logic: HIGH at timer-zero, LOW at comparator match → servo PWM.
 * ========================================================================== */
static void mcpwm_servo_init(void)
{
    ESP_LOGI(TAG, "Initialising MCPWM (50 Hz servo PWM)");

    /* Shared timer — 1 MHz resolution, 20 ms period */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks  = SERVO_TIMEBASE_PERIOD,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    /* --- Servo 1: Operator + Comparator + Generator --- */
    mcpwm_oper_handle_t oper1 = NULL;
    mcpwm_operator_config_t op1_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&op1_cfg, &oper1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer));

    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true, /* Update at timer-zero for glitch-free changes */
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &cmp_cfg, &s_servo1_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo1_cmp, angle_to_compare(0)));

    mcpwm_gen_handle_t gen1 = NULL;
    mcpwm_generator_config_t gen1_cfg = { .gen_gpio_num = SERVO1_GPIO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &gen1_cfg, &gen1));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_servo1_cmp,
                                       MCPWM_GEN_ACTION_LOW)));

    /* --- Servo 2: Operator + Comparator + Generator --- */
    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t op2_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&op2_cfg, &oper2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer));

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &cmp_cfg, &s_servo2_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo2_cmp, angle_to_compare(0)));

    mcpwm_gen_handle_t gen2 = NULL;
    mcpwm_generator_config_t gen2_cfg = { .gen_gpio_num = SERVO2_GPIO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &gen2_cfg, &gen2));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_servo2_cmp,
                                       MCPWM_GEN_ACTION_LOW)));

    /* Enable and start the shared timer */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM ready — Servo1:GPIO%d Servo2:GPIO%d",
             SERVO1_GPIO, SERVO2_GPIO);
}

/* ==========================================================================
 *  I2S audio output initialisation
 *
 *  Configures I2S in standard TX (Phillips/standard I2S) mode at 8 kHz,
 *  16-bit mono. This is compatible with I2S digital-input amplifier modules
 *  such as the MAX98357A, PAM8302 (if digital), etc.
 *
 *  For analogue-input Arduino amp modules:
 *    Add an RC low-pass filter on I2S_DOUT_GPIO (R=10kΩ, C=10nF, fc≈1.6kHz)
 *    to convert the 1-bit PDM/I2S stream to a smooth analogue waveform.
 * ========================================================================== */
static void i2s_audio_init(void)
{
    ESP_LOGI(TAG, "Initialising I2S audio output at %d Hz, 16-bit mono",
             AUDIO_SAMPLE_RATE_HZ);

    /* Create the I2S TX channel on I2S port 0 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0,      /* I2S port number — ESP32-S3 has I2S0 and I2S1     */
        I2S_ROLE_MASTER /* This device drives the clocks                     */
    );
    /* DMA buffer: 4 buffers × 512 frames — balances latency and smoothness */
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = 512;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx_handle, NULL));

    /* Configure standard I2S mode (Phillips format) */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            AUDIO_BITS_PER_SAMPLE,  /* 16-bit samples                        */
            I2S_SLOT_MODE_MONO      /* Single channel (mono audio)           */
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    /* Master clock not needed for most amps */
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_LRCLK_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,    /* No audio input on the receiver    */
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx_handle));

    ESP_LOGI(TAG, "I2S ready — BCLK:GPIO%d LR:GPIO%d DOUT:GPIO%d",
             I2S_BCLK_GPIO, I2S_LRCLK_GPIO, I2S_DOUT_GPIO);
}

/* ==========================================================================
 *  Audio playback task
 *
 *  Blocks on PLAY_EVENT_BIT. When signalled, converts the stored 8-bit
 *  unsigned PCM buffer to 16-bit signed PCM (required by I2S), then writes
 *  it to the I2S peripheral in chunks.
 *
 *  8-bit unsigned PCM → 16-bit signed PCM conversion:
 *    uint8  value range: 0–255, silence at 128
 *    int16  value range: –32768 to +32767, silence at 0
 *    Formula: int16 = ((uint8 - 128) * 256)
 * ========================================================================== */
static void playback_task(void *pvParameters)
{
    /* Temporary buffer for 16-bit upscaled PCM.
     * Process CHUNK samples at a time to avoid large stack allocations.     */
    const uint32_t PLAY_CHUNK = 256;
    int16_t *pcm16 = (int16_t *)malloc(PLAY_CHUNK * sizeof(int16_t));
    if (!pcm16) {
        ESP_LOGE(TAG, "Failed to allocate playback chunk buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Playback task ready — waiting for play command...");

    while (true) {
        /* Block here until the ESP-NOW callback sets the PLAY_EVENT_BIT */
        xEventGroupWaitBits(s_play_event_group,
                            PLAY_EVENT_BIT,
                            pdTRUE,    /* Clear the bit after waking          */
                            pdFALSE,   /* Wait for any bit (only one bit used)*/
                            portMAX_DELAY);

        /* Snapshot audio buffer info under mutex to avoid race conditions */
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        uint32_t num_samples = s_audio_samples;
        xSemaphoreGive(s_audio_mutex);

        if (num_samples == 0) {
            ESP_LOGW(TAG, "Play command received but no audio stored — ignoring");
            continue;
        }

        ESP_LOGI(TAG, "Playing %lu samples (%.2f s) via I2S",
                 num_samples, (float)num_samples / AUDIO_SAMPLE_RATE_HZ);

        /* Write audio to I2S in PLAY_CHUNK-sample chunks */
        uint32_t offset = 0;
        while (offset < num_samples) {
            uint32_t remaining = num_samples - offset;
            uint32_t chunk     = (remaining > PLAY_CHUNK) ? PLAY_CHUNK : remaining;

            /* Convert 8-bit unsigned → 16-bit signed PCM */
            for (uint32_t i = 0; i < chunk; i++) {
                int16_t val = ((int16_t)s_audio_buf[offset + i] - 128) * 256;
                pcm16[i] = val;
            }

            /* Write to I2S DMA — blocks until all bytes are accepted */
            size_t bytes_written = 0;
            esp_err_t ret = i2s_channel_write(
                s_i2s_tx_handle,
                pcm16,
                chunk * sizeof(int16_t),
                &bytes_written,
                pdMS_TO_TICKS(1000)  /* Generous timeout: 1 s               */
            );

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "I2S write error at sample %lu: %s",
                         offset, esp_err_to_name(ret));
                break;
            }

            offset += chunk;
        }

        ESP_LOGI(TAG, "Playback complete");
    }

    free(pcm16);
    vTaskDelete(NULL);
}

/* ==========================================================================
 *  Servo update task
 *
 *  Receives servo_command structs from s_servo_cmd_queue and updates MCPWM.
 * ========================================================================== */
static void servo_task(void *pvParameters)
{
    pkt_servo_t cmd;
    ESP_LOGI(TAG, "Servo task started");

    while (true) {
        if (xQueueReceive(s_servo_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Servo: ch1=%d° ch2=%d°",
                     cmd.servo1_angle, cmd.servo2_angle);

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
                s_servo1_cmp, angle_to_compare(cmd.servo1_angle)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
                s_servo2_cmp, angle_to_compare(cmd.servo2_angle)));
        }
    }
    vTaskDelete(NULL);
}

/* ==========================================================================
 *  ESP-NOW receive callback
 *
 *  Called from the Wi-Fi task context — keep work minimal.
 *  Routes packets by pkt_type byte:
 *    PKT_TYPE_SERVO      → enqueue servo command
 *    PKT_TYPE_AUDIO_START → initialise audio receive state
 *    PKT_TYPE_AUDIO_FRAG → copy audio payload into s_audio_buf
 *    PKT_TYPE_AUDIO_PLAY → set event bit to wake the playback task
 * ========================================================================== */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len)
{
    if (len < 1) return; /* Sanity check — at minimum we need the type byte */

    uint8_t pkt_type = data[0];

    switch (pkt_type) {

        /* ---------------------------------------------------------------- */
        case PKT_TYPE_SERVO: {
            if (len < (int)sizeof(pkt_servo_t)) break;
            pkt_servo_t cmd;
            memcpy(&cmd, data, sizeof(pkt_servo_t));
            /* Send to servo task — discard if queue full (control is real-time) */
            BaseType_t hp = pdFALSE;
            xQueueSendFromISR(s_servo_cmd_queue, &cmd, &hp);
            portYIELD_FROM_ISR(hp);
            break;
        }

        /* ---------------------------------------------------------------- */
        case PKT_TYPE_AUDIO_START: {
            if (len < (int)sizeof(pkt_audio_start_t)) break;
            pkt_audio_start_t *p = (pkt_audio_start_t *)data;

            xSemaphoreTakeFromISR(s_audio_mutex, NULL);

            /* Clamp to our buffer size */
            s_audio_samples  = (p->total_samples <= AUDIO_MAX_SAMPLES)
                               ? p->total_samples : AUDIO_MAX_SAMPLES;
            s_expected_frags = p->total_frags;
            s_received_frags = 0;

            /* Zero out the destination buffer for this transfer */
            memset(s_audio_buf, 128, s_audio_samples); /* 128 = silence in 8-bit unsigned */

            xSemaphoreGiveFromISR(s_audio_mutex, NULL);

            ESP_EARLY_LOGI(TAG, "Audio transfer started: %lu samples, %u frags",
                           p->total_samples, p->total_frags);
            break;
        }

        /* ---------------------------------------------------------------- */
        case PKT_TYPE_AUDIO_FRAG: {
            if (len < (int)(sizeof(pkt_audio_frag_t) - AUDIO_CHUNK_BYTES + 1)) break;
            pkt_audio_frag_t *frag = (pkt_audio_frag_t *)data;

            /* Calculate destination offset in the audio buffer */
            uint32_t offset = (uint32_t)frag->frag_index * AUDIO_CHUNK_BYTES;

            xSemaphoreTakeFromISR(s_audio_mutex, NULL);

            if (offset + frag->payload_len <= s_audio_samples) {
                memcpy(s_audio_buf + offset, frag->audio, frag->payload_len);
                s_received_frags++;

                if (s_received_frags >= s_expected_frags) {
                    ESP_EARLY_LOGI(TAG, "Audio transfer complete: all %u frags received",
                                   s_expected_frags);
                }
            } else {
                ESP_EARLY_LOGW(TAG, "Fragment %u out of range — discarded",
                               frag->frag_index);
            }

            xSemaphoreGiveFromISR(s_audio_mutex, NULL);
            break;
        }

        /* ---------------------------------------------------------------- */
        case PKT_TYPE_AUDIO_PLAY: {
            /* Wake the playback task by setting the event bit */
            BaseType_t hp = pdFALSE;
            xEventGroupSetBitsFromISR(s_play_event_group, PLAY_EVENT_BIT, &hp);
            portYIELD_FROM_ISR(hp);
            ESP_EARLY_LOGI(TAG, "Play command received");
            break;
        }

        default:
            ESP_EARLY_LOGW(TAG, "Unknown packet type 0x%02X — ignored", pkt_type);
            break;
    }
}

/* ==========================================================================
 *  Wi-Fi + ESP-NOW initialisation
 * ========================================================================== */
static void wifi_espnow_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Receiver MAC: " MACSTR, MAC2STR(mac));
    ESP_LOGI(TAG, ">>> Paste the above MAC into RECEIVER_MAC_ADDR in controller main.c <<<");

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW initialised — listening for packets");
}

/* ==========================================================================
 *  app_main
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "  ESP32-S3 Puppet Receiver");
    ESP_LOGI(TAG, "  ESP-IDF v6.0.1");
    ESP_LOGI(TAG, "================================");

    /* --- Allocate audio buffer --- */
    s_audio_buf = (uint8_t *)heap_caps_malloc(
        AUDIO_MAX_SAMPLES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!s_audio_buf) {
        ESP_LOGW(TAG, "PSRAM not available — using internal RAM for audio buffer");
        s_audio_buf = (uint8_t *)malloc(AUDIO_MAX_SAMPLES);
    }
    if (!s_audio_buf) {
        ESP_LOGE(TAG, "FATAL: Cannot allocate %d byte audio buffer", AUDIO_MAX_SAMPLES);
        return;
    }
    memset(s_audio_buf, 128, AUDIO_MAX_SAMPLES); /* Pre-fill with silence    */
    ESP_LOGI(TAG, "Audio buffer: %d bytes", AUDIO_MAX_SAMPLES);

    /* --- Create synchronisation primitives --- */
    s_audio_mutex      = xSemaphoreCreateMutex();
    s_play_event_group = xEventGroupCreate();
    s_servo_cmd_queue  = xQueueCreate(10, sizeof(pkt_servo_t));

    /* --- Peripheral initialisation --- */
    mcpwm_servo_init();
    i2s_audio_init();
    wifi_espnow_init();

    /* --- Start tasks --- */
    /* Servo update task — high priority for responsive motor control */
    xTaskCreatePinnedToCore(servo_task,    "servo",    4096, NULL, 6, NULL, 1);

    /* Playback task — medium priority; audio is non-time-critical vs servos */
    xTaskCreatePinnedToCore(playback_task, "playback", 8192, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Receiver ready — awaiting servo and audio commands");
}