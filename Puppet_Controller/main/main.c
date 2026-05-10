/*
 * =============================================================================
 *  CONTROLLER FIRMWARE — ESP32-S3-Nano Development Board
 *  Framework : ESP-IDF v6.0.1
 *
 *  Features:
 *    1. Read two potentiometers → send servo angle commands via ESP-NOW.
 *    2. RECORD button (GPIO 8):
 *         • First press  → suspends servo task, starts recording from MAX9814
 *                          microphone via ADC continuous DMA driver.
 *                          LED (GPIO 7) flashes during recording.
 *         • Second press → stops recording; ADC1 is released; servo task is
 *                          resumed; audio buffer is automatically fragmented
 *                          and sent to the receiver via ESP-NOW.
 *    3. PLAY button (GPIO 9):
 *         • Sends a "play" command packet to the receiver, which then replays
 *           the last received recording through its speaker amplifier.
 *
 *  ADC conflict note:
 *    The ESP32-S3 ADC1 cannot be shared between the oneshot driver (used for
 *    potentiometers) and the continuous DMA driver (used for the microphone).
 *    ADC2 is also unavailable because Wi-Fi occupies it. The fix is to suspend
 *    the servo task (which calls adc_oneshot_read) before starting the
 *    continuous driver, then resume it after stopping. The MCPWM peripheral
 *    continues driving the servos at their last position in hardware while
 *    the task is suspended — no physical movement occurs during recording.
 *
 *  Hardware wiring (adjust GPIO numbers to match your physical board layout):
 *    Potentiometer 1 wiper  → GPIO 1  (ADC1_CH0)
 *    Potentiometer 2 wiper  → GPIO 2  (ADC1_CH1)
 *    MAX9814 OUT            → GPIO 4  (ADC1_CH3)
 *    MAX9814 VDD            → 3.3 V
 *    MAX9814 GND            → GND
 *    MAX9814 GAIN           → leave floating (middle gain ~40 dB)
 *    Record button          → GPIO 8  (pulled HIGH internally; press = LOW)
 *    Play button            → GPIO 9  (pulled HIGH internally; press = LOW)
 *    Recording LED          → GPIO 7  (active HIGH; add 330 Ω series resistor)
 *
 *  Audio parameters:
 *    Sample rate  : 8 000 Hz  (telephone quality — keeps buffer sizes small)
 *    Bit depth    : 8-bit unsigned PCM (ADC 12-bit values scaled to 0–255)
 *    Max recording: AUDIO_MAX_SAMPLES = 8000 * MAX_RECORD_SECONDS
 *
 *  ESP-NOW fragmentation:
 *    Each ESP-NOW packet carries up to AUDIO_CHUNK_BYTES of audio payload plus
 *    a small header (packet index, total packets, type flag).
 *    The receiver reassembles them in order before playback.
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
#include "driver/gpio.h"

/* ADC continuous (DMA) driver — used for high-speed audio sampling          */
#include "esp_adc/adc_continuous.h"
/* ADC oneshot driver — used for potentiometer reads                         */
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* -------------------------------------------------------------------------- */
/*  Logging tag                                                               */
/* -------------------------------------------------------------------------- */
static const char *TAG = "CTRL";

/* ==========================================================================
 *  USER CONFIGURATION — edit these to match your hardware
 * ========================================================================== */

/* --- ESP-NOW peer --- */
/* Replace with your receiver board's MAC address (printed on first boot)    */
static const uint8_t RECEIVER_MAC_ADDR[6] = {0xDC, 0xB4, 0xD9, 0x25, 0x79, 0xDC};

/* --- GPIO assignments --- */
#define POT1_GPIO           1       /* Servo 1 potentiometer (ADC1_CH0)      */
#define POT2_GPIO           2       /* Servo 2 potentiometer (ADC1_CH1)      */
#define MIC_GPIO            4       /* MAX9814 OUT pin      (ADC1_CH3)        */
#define LED_RECORD_GPIO     7       /* Recording indicator LED               */
#define BTN_RECORD_GPIO     8       /* Record start/stop button              */
#define BTN_PLAY_GPIO       9       /* Play recorded audio button            */

/* --- ADC channels --- */
#define POT1_ADC_CHANNEL    ADC_CHANNEL_0   /* GPIO 1 */
#define POT2_ADC_CHANNEL    ADC_CHANNEL_1   /* GPIO 2 */
#define MIC_ADC_CHANNEL     ADC_CHANNEL_3   /* GPIO 4 */

/* --- Audio parameters --- */
#define AUDIO_SAMPLE_RATE_HZ    8000    /* 8 kHz — adequate for voice        */
#define MAX_RECORD_SECONDS      5       /* Maximum recording length          */
#define AUDIO_MAX_SAMPLES       (AUDIO_SAMPLE_RATE_HZ * MAX_RECORD_SECONDS)
                                        /* = 40 000 bytes @ 8-bit            */

/* --- ESP-NOW fragmentation ---
 * Max ESP-NOW payload = 250 bytes. We use a small header, leaving
 * AUDIO_CHUNK_BYTES bytes of audio data per fragment.                       */
#define AUDIO_CHUNK_BYTES       200     /* Audio bytes per ESP-NOW packet    */

/* --- Servo angle range --- */
#define SERVO_MIN_DEGREE    (-90)
#define SERVO_MAX_DEGREE    90

/* --- Servo command send interval --- */
#define SERVO_SEND_INTERVAL_MS  50      /* 20 Hz update rate                 */

/* --- Button debounce --- */
#define DEBOUNCE_MS             50

/* ==========================================================================
 *  Packet type identifiers
 *  These are used in the first byte of every ESP-NOW packet so the receiver
 *  can distinguish servo commands, audio fragments, and play commands.
 * ========================================================================== */
#define PKT_TYPE_SERVO          0x01
#define PKT_TYPE_AUDIO_FRAG     0x02
#define PKT_TYPE_AUDIO_PLAY     0x03
#define PKT_TYPE_AUDIO_START    0x04    /* Signals start of a new transfer   */

/* ==========================================================================
 *  Packet structures
 *  IMPORTANT: All structures must be identical in controller and receiver.
 * ========================================================================== */

/* Servo angle command packet */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;          /* PKT_TYPE_SERVO                            */
    int16_t  servo1_angle;      /* –90 to +90 degrees                        */
    int16_t  servo2_angle;      /* –90 to +90 degrees                        */
} pkt_servo_t;

/* Audio fragment packet — carries a chunk of recorded audio PCM data */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;          /* PKT_TYPE_AUDIO_FRAG                       */
    uint16_t frag_index;        /* Fragment sequence number (0-based)        */
    uint16_t total_frags;       /* Total number of fragments in this transfer*/
    uint16_t payload_len;       /* Number of valid audio bytes in this packet*/
    uint8_t  audio[AUDIO_CHUNK_BYTES]; /* Raw 8-bit PCM audio samples        */
} pkt_audio_frag_t;

/* Audio start packet — sent first to tell receiver total audio length */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;          /* PKT_TYPE_AUDIO_START                      */
    uint32_t total_samples;     /* Total number of 8-bit PCM samples         */
    uint16_t total_frags;       /* Total number of fragments that will follow*/
} pkt_audio_start_t;

/* Play command packet — instructs receiver to replay stored audio */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;          /* PKT_TYPE_AUDIO_PLAY                       */
} pkt_play_t;

/* ==========================================================================
 *  Module-level state
 * ========================================================================== */

/* ADC handles */
static adc_oneshot_unit_handle_t  s_adc_pot_handle  = NULL;  /* Potentiometers */
static adc_continuous_handle_t    s_adc_mic_handle  = NULL;  /* Microphone     */
static adc_cali_handle_t          s_cali_pot1       = NULL;
static adc_cali_handle_t          s_cali_pot2       = NULL;

/* Audio recording buffer — allocated once at startup in SPIRAM if available */
static uint8_t  *s_audio_buf       = NULL;  /* 8-bit PCM samples             */
static uint32_t  s_audio_samples   = 0;     /* Number of samples captured    */
static bool      s_is_recording    = false; /* True while recording in progress */

/* Semaphore protecting s_is_recording and s_audio_* from concurrent access */
static SemaphoreHandle_t s_audio_mutex = NULL;

/*
 * Servo task handle — stored so recording_task can suspend and resume it.
 *
 * Why this is necessary:
 *   ADC1 cannot be shared between adc_oneshot (potentiometers) and
 *   adc_continuous (microphone) at the same time. ADC2 is unavailable
 *   because Wi-Fi holds it. Suspending the servo task prevents any
 *   adc_oneshot_read() calls from occurring while the continuous driver
 *   owns ADC1. The MCPWM hardware continues outputting the last PWM signal
 *   autonomously, so the physical servos hold position during recording.
 */
static TaskHandle_t s_servo_task_handle = NULL;

/* ==========================================================================
 *  Helper: map millivolt reading (0–3300 mV) to servo angle (–90° to +90°)
 * ========================================================================== */
static int16_t mv_to_angle(int voltage_mv)
{
    if (voltage_mv < 0)    voltage_mv = 0;
    if (voltage_mv > 3300) voltage_mv = 3300;
    return (int16_t)(
        (voltage_mv * (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)) / 3300
        + SERVO_MIN_DEGREE
    );
}

/* ==========================================================================
 *  Helper: read potentiometer ADC channel and return millivolts
 * ========================================================================== */
static int pot_read_mv(adc_channel_t ch, adc_cali_handle_t cali)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc_pot_handle, ch, &raw));
    if (cali) {
        int mv = 0;
        adc_cali_raw_to_voltage(cali, raw, &mv);
        return mv;
    }
    return (raw * 3300) / 4095;
}

/* ==========================================================================
 *  ESP-NOW send callback (v6 API — uses esp_now_send_info_t)
 * ========================================================================== */
static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW TX failed to " MACSTR,
                 MAC2STR(tx_info->des_addr));
    }
}

/* ==========================================================================
 *  Wi-Fi + ESP-NOW initialisation
 * ========================================================================== */
static void wifi_espnow_init(void)
{
    /* NVS flash — required by Wi-Fi driver */
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
    ESP_LOGI(TAG, "Controller MAC: " MACSTR, MAC2STR(mac));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    /* Register receiver as a peer */
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, RECEIVER_MAC_ADDR, 6);
    peer.channel = 0;
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_LOGI(TAG, "Receiver peer registered: " MACSTR, MAC2STR(RECEIVER_MAC_ADDR));
}

/* ==========================================================================
 *  Potentiometer ADC initialisation (oneshot driver)
 * ========================================================================== */
static void pot_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_pot_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_pot_handle, POT1_ADC_CHANNEL, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_pot_handle, POT2_ADC_CHANNEL, &ch_cfg));

    /* Calibration — gracefully skipped if eFuse data not present */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_pot1) != ESP_OK)
        s_cali_pot1 = NULL;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_pot2) != ESP_OK)
        s_cali_pot2 = NULL;

    ESP_LOGI(TAG, "Potentiometer ADC (oneshot) initialised");
}

/* ==========================================================================
 *  Microphone ADC initialisation (continuous DMA driver)
 *
 *  The MAX9814 outputs an analogue voltage on its OUT pin proportional to the
 *  microphone signal, centred around VDD/2 (~1.65 V). We sample this with the
 *  ESP32-S3 ADC1 continuous driver at AUDIO_SAMPLE_RATE_HZ using DMA so that
 *  samples are not missed even when the CPU is busy.
 *
 *  This function only creates and configures the handle. The driver is NOT
 *  started here — it is started inside recording_task() after the servo task
 *  has been suspended, avoiding the ADC1 conflict.
 * ========================================================================== */
static void mic_adc_init(void)
{
    /* DMA buffer: hold enough bytes for ~100 ms of audio at 8 kHz.
     * Each DMA result is 4 bytes (adc_digi_output_data_t type2 format).    */
    uint32_t buf_size = (AUDIO_SAMPLE_RATE_HZ / 10) * sizeof(adc_digi_output_data_t);

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = buf_size * 4,  /* Internal ring buffer          */
        .conv_frame_size    = buf_size,      /* Bytes per DMA interrupt       */
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &s_adc_mic_handle));

    /* Configure the microphone channel */
    adc_digi_pattern_config_t pat = {
        .atten      = ADC_ATTEN_DB_12,
        .channel    = MIC_ADC_CHANNEL,
        .unit       = ADC_UNIT_1,
        .bit_width  = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t cont_cfg = {
        .sample_freq_hz = AUDIO_SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num    = 1,
        .adc_pattern    = &pat,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc_mic_handle, &cont_cfg));
    ESP_LOGI(TAG, "Microphone ADC (continuous DMA) configured at %d Hz",
             AUDIO_SAMPLE_RATE_HZ);
}

/* ==========================================================================
 *  LED + Button GPIO initialisation
 * ========================================================================== */
static void gpio_init(void)
{
    /* Recording status LED — output */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_RECORD_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));
    gpio_set_level(LED_RECORD_GPIO, 0); /* LED off initially */

    /* Buttons — input with internal pull-up; press connects pin to GND */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BTN_RECORD_GPIO) | (1ULL << BTN_PLAY_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));
    ESP_LOGI(TAG, "GPIO initialised — LED:%d  RecBtn:%d  PlayBtn:%d",
             LED_RECORD_GPIO, BTN_RECORD_GPIO, BTN_PLAY_GPIO);
}

/* ==========================================================================
 *  LED flash task — runs while s_is_recording is true
 *
 *  Toggles the recording LED every 250 ms. Deletes itself once recording
 *  stops and ensures the LED is left in the OFF state.
 * ========================================================================== */
static void led_flash_task(void *pvParameters)
{
    bool led_state = false;
    while (true) {
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        bool still_recording = s_is_recording;
        xSemaphoreGive(s_audio_mutex);

        if (!still_recording) {
            gpio_set_level(LED_RECORD_GPIO, 0); /* Ensure LED is off on exit */
            vTaskDelete(NULL);
            return;
        }

        led_state = !led_state;
        gpio_set_level(LED_RECORD_GPIO, led_state ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(250)); /* Flash at 2 Hz */
    }
}

/* ==========================================================================
 *  Send recorded audio to receiver via ESP-NOW (fragmented)
 *
 *  Called from recording_task() after recording stops. Sends:
 *    1. A PKT_TYPE_AUDIO_START packet with total sample count + fragment count.
 *    2. N × PKT_TYPE_AUDIO_FRAG packets, each with AUDIO_CHUNK_BYTES of PCM.
 *
 *  A short delay between packets prevents the ESP-NOW TX queue overflowing.
 * ========================================================================== */
static void send_audio_to_receiver(const uint8_t *audio_data, uint32_t num_samples)
{
    if (num_samples == 0) {
        ESP_LOGW(TAG, "Nothing to send — recording is empty");
        return;
    }

    uint16_t total_frags = (uint16_t)((num_samples + AUDIO_CHUNK_BYTES - 1)
                                       / AUDIO_CHUNK_BYTES);

    ESP_LOGI(TAG, "Sending %lu samples in %u fragments", num_samples, total_frags);

    /* Step 1: Send START packet so receiver can allocate its buffer */
    pkt_audio_start_t start_pkt = {
        .pkt_type      = PKT_TYPE_AUDIO_START,
        .total_samples = num_samples,
        .total_frags   = total_frags,
    };
    esp_now_send(RECEIVER_MAC_ADDR, (uint8_t *)&start_pkt, sizeof(start_pkt));
    vTaskDelay(pdMS_TO_TICKS(20)); /* Give receiver time to prepare          */

    /* Step 2: Send each audio fragment */
    pkt_audio_frag_t frag_pkt;
    uint32_t offset = 0;

    for (uint16_t i = 0; i < total_frags; i++) {
        uint32_t remaining = num_samples - offset;
        uint16_t chunk_len = (remaining >= AUDIO_CHUNK_BYTES)
                             ? AUDIO_CHUNK_BYTES
                             : (uint16_t)remaining;

        frag_pkt.pkt_type    = PKT_TYPE_AUDIO_FRAG;
        frag_pkt.frag_index  = i;
        frag_pkt.total_frags = total_frags;
        frag_pkt.payload_len = chunk_len;
        memcpy(frag_pkt.audio, audio_data + offset, chunk_len);

        esp_err_t err = esp_now_send(
            RECEIVER_MAC_ADDR,
            (uint8_t *)&frag_pkt,
            sizeof(frag_pkt) - AUDIO_CHUNK_BYTES + chunk_len
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Fragment %u/%u failed: %s",
                     i + 1, total_frags, esp_err_to_name(err));
        } else {
            ESP_LOGD(TAG, "Fragment %u/%u sent (%u bytes)",
                     i + 1, total_frags, chunk_len);
        }

        offset += chunk_len;

        /* 10 ms inter-packet gap prevents ESP-NOW TX queue overflow         */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Audio transfer complete — %lu samples in %u fragments",
             num_samples, total_frags);
}

/* ==========================================================================
 *  Recording task
 *
 *  Lifecycle:
 *    1. Suspend servo_task to release ADC1 from oneshot driver.
 *    2. Start ADC1 continuous driver to capture microphone audio.
 *    3. Read DMA frames and fill s_audio_buf until stop is requested or
 *       MAX_RECORD_SECONDS is reached.
 *    4. Stop ADC1 continuous driver.
 *    5. Resume servo_task so potentiometer reads can continue.
 *    6. Call send_audio_to_receiver() to transmit the captured audio.
 *    7. Self-delete.
 * ========================================================================== */
static void recording_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Recording task started");

    /*
     * STEP 1 — Suspend the servo task.
     *
     * The servo task calls adc_oneshot_read() on ADC1. The continuous DMA
     * driver also owns ADC1 during recording. They cannot coexist, so we
     * suspend the servo task here before starting the continuous driver.
     * The MCPWM peripheral keeps the servo PWM signals running in hardware
     * at their last commanded position — the servos physically hold still.
     */
    if (s_servo_task_handle != NULL) {
        vTaskSuspend(s_servo_task_handle);
        ESP_LOGI(TAG, "Servo task suspended — servos holding last position");
    }

    /* STEP 2 — Start the ADC continuous driver */
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_mic_handle));
    ESP_LOGI(TAG, "ADC continuous driver started — capturing audio...");

    /* Allocate a temporary DMA read buffer on the heap (not stack) */
    uint32_t frame_bytes = (AUDIO_SAMPLE_RATE_HZ / 10) * sizeof(adc_digi_output_data_t);
    uint8_t *dma_buf = (uint8_t *)malloc(frame_bytes);
    if (!dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA read buffer — aborting recording");
        adc_continuous_stop(s_adc_mic_handle);
        if (s_servo_task_handle != NULL) vTaskResume(s_servo_task_handle);
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        s_is_recording = false;
        xSemaphoreGive(s_audio_mutex);
        vTaskDelete(NULL);
        return;
    }

    s_audio_samples = 0;

    /* STEP 3 — Main capture loop */
    while (true) {
        /* Check if the user pressed the stop button */
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        bool keep_recording = s_is_recording;
        xSemaphoreGive(s_audio_mutex);

        if (!keep_recording) break;

        /* Auto-stop if the maximum recording length is reached */
        if (s_audio_samples >= AUDIO_MAX_SAMPLES) {
            ESP_LOGW(TAG, "Max recording length (%d s) reached — stopping automatically",
                     MAX_RECORD_SECONDS);
            xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
            s_is_recording = false;
            xSemaphoreGive(s_audio_mutex);
            break;
        }

        /* Read one DMA frame from the ADC ring buffer */
        uint32_t out_len = 0;
        esp_err_t ret = adc_continuous_read(
            s_adc_mic_handle,
            dma_buf,
            frame_bytes,
            &out_len,
            10  /* Timeout ms — short so loop stays responsive to stop button */
        );

        if (ret == ESP_OK && out_len > 0) {
            /*
             * Each DMA result is a 4-byte adc_digi_output_data_t (TYPE2).
             * Extract the 12-bit ADC value and scale to 8-bit unsigned PCM.
             * The MAX9814 biases its output at VDD/2 (~1.65 V, ~ADC 2048),
             * which maps to 128 in 8-bit unsigned PCM — representing silence.
             */
            uint32_t num_results = out_len / sizeof(adc_digi_output_data_t);

            for (uint32_t j = 0; j < num_results; j++) {
                if (s_audio_samples >= AUDIO_MAX_SAMPLES) break;

                adc_digi_output_data_t *p =
                    (adc_digi_output_data_t *)(dma_buf + j * sizeof(adc_digi_output_data_t));

                /* Skip results from other channels that may appear in the buffer */
                if (p->type2.channel != MIC_ADC_CHANNEL) continue;

                /* Scale 12-bit (0–4095) → 8-bit unsigned (0–255) */
                s_audio_buf[s_audio_samples++] = (uint8_t)(p->type2.data >> 4);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            /* No data ready yet — normal during quiet periods; loop again   */
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(ret));
        }
    }

    /* STEP 4 — Stop the ADC continuous driver */
    adc_continuous_stop(s_adc_mic_handle);
    free(dma_buf);
    ESP_LOGI(TAG, "Recording stopped — %lu samples captured (%.2f s)",
             s_audio_samples,
             (float)s_audio_samples / AUDIO_SAMPLE_RATE_HZ);

    /*
     * STEP 5 — Resume the servo task.
     *
     * ADC1 is now free. The oneshot driver can service potentiometer reads
     * again without conflicting with the continuous driver.
     */
    if (s_servo_task_handle != NULL) {
        vTaskResume(s_servo_task_handle);
        ESP_LOGI(TAG, "Servo task resumed — potentiometer control restored");
    }

    /* STEP 6 — Transmit the captured audio to the receiver */
    send_audio_to_receiver(s_audio_buf, s_audio_samples);

    /* STEP 7 — Clean up */
    vTaskDelete(NULL);
}

/* ==========================================================================
 *  Button polling task
 *
 *  Polls both buttons at ~50 Hz with software debouncing.
 *    Record button: toggles recording on/off
 *    Play button:   sends a play command packet to the receiver
 * ========================================================================== */
static void button_task(void *pvParameters)
{
    bool prev_record_btn = true; /* true = released (pin pulled high) */
    bool prev_play_btn   = true;

    while (true) {
        bool record_btn = (gpio_get_level(BTN_RECORD_GPIO) == 1);
        bool play_btn   = (gpio_get_level(BTN_PLAY_GPIO)   == 1);

        /* --- Record button: detect falling edge (press) --- */
        if (prev_record_btn && !record_btn) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

            if (gpio_get_level(BTN_RECORD_GPIO) == 0) {
                xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                bool currently_recording = s_is_recording;
                xSemaphoreGive(s_audio_mutex);

                if (!currently_recording) {
                    /* ---- START RECORDING ---- */
                    ESP_LOGI(TAG, "Record button — STARTING recording");
                    s_audio_samples = 0;

                    xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                    s_is_recording = true;
                    xSemaphoreGive(s_audio_mutex);

                    /* Start the LED flash indicator */
                    xTaskCreatePinnedToCore(led_flash_task, "led_flash",
                                            2048, NULL, 3, NULL, 1);

                    /* Start the recording task */
                    xTaskCreatePinnedToCore(recording_task, "recording",
                                            8192, NULL, 6, NULL, 1);
                } else {
                    /* ---- STOP RECORDING ---- */
                    ESP_LOGI(TAG, "Record button — STOPPING recording");
                    xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                    s_is_recording = false;
                    xSemaphoreGive(s_audio_mutex);
                    /* recording_task detects this flag and handles the rest */
                }
            }
        }

        /* --- Play button: detect falling edge (press) --- */
        if (prev_play_btn && !play_btn) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

            if (gpio_get_level(BTN_PLAY_GPIO) == 0) {
                ESP_LOGI(TAG, "Play button — sending play command to receiver");

                pkt_play_t play_pkt = { .pkt_type = PKT_TYPE_AUDIO_PLAY };
                esp_err_t err = esp_now_send(
                    RECEIVER_MAC_ADDR,
                    (uint8_t *)&play_pkt,
                    sizeof(play_pkt)
                );
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Play command failed: %s", esp_err_to_name(err));
                }
            }
        }

        prev_record_btn = record_btn;
        prev_play_btn   = play_btn;

        vTaskDelay(pdMS_TO_TICKS(20)); /* Poll at ~50 Hz */
    }
}

/* ==========================================================================
 *  Servo command task
 *
 *  Reads both potentiometers, maps to servo angles, and broadcasts a servo
 *  command packet to the receiver at SERVO_SEND_INTERVAL_MS intervals.
 *
 *  This task is suspended by recording_task() while ADC1 is owned by the
 *  continuous driver. It resumes automatically after recording stops.
 * ========================================================================== */
static void servo_task(void *pvParameters)
{
    pkt_servo_t cmd;
    cmd.pkt_type = PKT_TYPE_SERVO;

    ESP_LOGI(TAG, "Servo task started");

    while (true) {
        int mv1 = pot_read_mv(POT1_ADC_CHANNEL, s_cali_pot1);
        int mv2 = pot_read_mv(POT2_ADC_CHANNEL, s_cali_pot2);

        cmd.servo1_angle = mv_to_angle(mv1);
        cmd.servo2_angle = mv_to_angle(mv2);

        ESP_LOGD(TAG, "Servo: ch1=%d° ch2=%d°", cmd.servo1_angle, cmd.servo2_angle);

        esp_now_send(RECEIVER_MAC_ADDR, (uint8_t *)&cmd, sizeof(cmd));

        vTaskDelay(pdMS_TO_TICKS(SERVO_SEND_INTERVAL_MS));
    }
}

/* ==========================================================================
 *  app_main
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "  ESP32-S3 Puppet Controller");
    ESP_LOGI(TAG, "  ESP-IDF v6.0.1");
    ESP_LOGI(TAG, "================================");

    /* --- Allocate audio recording buffer ---
     * Prefer PSRAM to avoid exhausting internal heap.                       */
    s_audio_buf = (uint8_t *)heap_caps_malloc(
        AUDIO_MAX_SAMPLES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!s_audio_buf) {
        ESP_LOGW(TAG, "PSRAM unavailable — allocating audio buffer in internal RAM");
        s_audio_buf = (uint8_t *)malloc(AUDIO_MAX_SAMPLES);
    }
    if (!s_audio_buf) {
        ESP_LOGE(TAG, "FATAL: Cannot allocate %d byte audio buffer", AUDIO_MAX_SAMPLES);
        return;
    }
    ESP_LOGI(TAG, "Audio buffer: %d bytes (%.1f s @ %d Hz)",
             AUDIO_MAX_SAMPLES, (float)MAX_RECORD_SECONDS, AUDIO_SAMPLE_RATE_HZ);

    /* Create mutex protecting recording state */
    s_audio_mutex = xSemaphoreCreateMutex();

    /* Peripheral initialisation */
    gpio_init();
    pot_adc_init();
    mic_adc_init();
    wifi_espnow_init();

    /*
     * Start servo_task and store its handle in s_servo_task_handle.
     * The handle is used by recording_task() to suspend/resume the task
     * around ADC1 continuous driver usage.
     */
    xTaskCreatePinnedToCore(servo_task,  "servo",   4096, NULL, 5,
                            &s_servo_task_handle,   /* <-- store handle */
                            0);

    xTaskCreatePinnedToCore(button_task, "buttons", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Controller ready.");
    ESP_LOGI(TAG, "  GPIO %d = Record button (press to start/stop)", BTN_RECORD_GPIO);
    ESP_LOGI(TAG, "  GPIO %d = Play button   (press to play on receiver)", BTN_PLAY_GPIO);
    ESP_LOGI(TAG, "  GPIO %d = Recording LED (flashes while recording)", LED_RECORD_GPIO);
}