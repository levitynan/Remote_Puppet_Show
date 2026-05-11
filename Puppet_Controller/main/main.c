/*
 * =============================================================================
 *  CONTROLLER FIRMWARE — ESP32-S3-Nano Development Board
 *  Framework : ESP-IDF v6.0.1
 *  Protocol  : Bluetooth Low Energy — NimBLE Central (GAP Scanner / GATT Client)
 *
 *  Features:
 *    1. Scans for a BLE peripheral advertising as "PuppetReceiver".
 *    2. On connect: exchanges MTU (512 bytes) then discovers the Puppet GATT
 *       service and its three characteristics (servo, audio, play).
 *    3. Servo task: reads two potentiometers on ADC2 (oneshot) at 20 Hz and
 *       sends angle commands via Write-Without-Response to the servo chr.
 *    4. RECORD button (GPIO 8):
 *         • First press  → starts recording from MAX9814 via ADC1 continuous DMA.
 *                          LED (GPIO 7) flashes during recording.
 *         • Second press → stops recording; sends audio fragments to audio chr.
 *    5. PLAY button (GPIO 9):
 *         • Sends a play command packet to the play characteristic.
 *
 *  ADC layout (no WiFi conflict — BLE does not use the WiFi radio):
 *    ADC2 oneshot    — potentiometers on GPIO 11 (CH0) and GPIO 12 (CH1)
 *    ADC1 continuous — MAX9814 microphone on GPIO 4 (CH3)
 *    No ADC conflict → servo task never needs to be suspended.
 *
 *  BLE MTU:
 *    Preferred MTU is set to 512 bytes. After connecting the controller
 *    initiates MTU exchange so audio fragment packets (207 bytes each) fit
 *    in a single ATT Write-Command. The default MTU of 23 bytes would silently
 *    truncate every audio fragment, producing noise on the receiver.
 *
 *  Hardware wiring (adjust #defines below if your board differs):
 *    Potentiometer 1 wiper → GPIO 11 (ADC2_CH0)
 *    Potentiometer 2 wiper → GPIO 12 (ADC2_CH1)
 *    MAX9814 OUT           → GPIO 4  (ADC1_CH3)
 *    MAX9814 VDD           → 3.3 V
 *    MAX9814 GND           → GND
 *    Record button         → GPIO 8  (pulled HIGH; press = LOW)
 *    Play button           → GPIO 9  (pulled HIGH; press = LOW)
 *    Recording LED         → GPIO 7  (active HIGH; 330 Ω series resistor)
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "esp_central.h"   /* peer.c / misc.c utility API */

static const char *TAG = "CTRL";

/* ==========================================================================
 *  USER CONFIGURATION
 * ========================================================================== */

#define POT1_GPIO           11      /* ADC2_CH0 — potentiometer 1              */
#define POT2_GPIO           12      /* ADC2_CH1 — potentiometer 2              */
#define MIC_GPIO            4       /* ADC1_CH3 — MAX9814 OUT                  */
#define LED_RECORD_GPIO     7
#define BTN_RECORD_GPIO     8
#define BTN_PLAY_GPIO       9

#define POT1_ADC_CHANNEL    ADC_CHANNEL_0   /* GPIO 11 on ADC2 */
#define POT2_ADC_CHANNEL    ADC_CHANNEL_1   /* GPIO 12 on ADC2 */
#define MIC_ADC_CHANNEL     ADC_CHANNEL_3   /* GPIO 4  on ADC1 */

#define AUDIO_SAMPLE_RATE_HZ    8000
#define MAX_RECORD_SECONDS      5
#define AUDIO_MAX_SAMPLES       (AUDIO_SAMPLE_RATE_HZ * MAX_RECORD_SECONDS)
#define AUDIO_CHUNK_BYTES       200

#define SERVO_MIN_DEGREE        (-90)
#define SERVO_MAX_DEGREE        90
#define SERVO_SEND_INTERVAL_MS  50
#define DEBOUNCE_MS             50

#define RECEIVER_BLE_NAME       "PuppetReceiver"

/* Must be >= sizeof(pkt_audio_frag_t) + 3 (ATT header) = 210 bytes.
 * 512 gives comfortable headroom and is the NimBLE stack maximum.           */
#define BLE_PREFERRED_MTU       512

/* Peer memory pool sizes for peer.c service-discovery helper */
#define PEER_MAX_PEERS  1
#define PEER_MAX_SVCS   5
#define PEER_MAX_CHRS   10
#define PEER_MAX_DSCS   10

/* ==========================================================================
 *  Packet type identifiers  (must match receiver)
 * ========================================================================== */
#define PKT_TYPE_SERVO          0x01
#define PKT_TYPE_AUDIO_FRAG     0x02
#define PKT_TYPE_AUDIO_PLAY     0x03
#define PKT_TYPE_AUDIO_START    0x04

/* ==========================================================================
 *  Packet structures  (must be identical to receiver)
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
 *  GATT UUIDs  (must match receiver exactly)
 * ========================================================================== */
static const ble_uuid128_t puppet_svc_uuid =
    BLE_UUID128_INIT(0xAB, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t servo_chr_uuid =
    BLE_UUID128_INIT(0xAC, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t audio_chr_uuid =
    BLE_UUID128_INIT(0xAD, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t play_chr_uuid =
    BLE_UUID128_INIT(0xAE, 0x90, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12);

/* ==========================================================================
 *  Module-level state
 * ========================================================================== */
static adc_oneshot_unit_handle_t  s_adc_pot_handle = NULL;
static adc_continuous_handle_t    s_adc_mic_handle = NULL;
static adc_cali_handle_t          s_cali_pot1      = NULL;
static adc_cali_handle_t          s_cali_pot2      = NULL;

static uint8_t          *s_audio_buf     = NULL;
static uint32_t          s_audio_samples = 0;
static bool              s_is_recording  = false;
static SemaphoreHandle_t s_audio_mutex   = NULL;

static uint16_t s_conn_handle          = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_servo_chr_val_handle = 0;
static uint16_t s_audio_chr_val_handle = 0;
static uint16_t s_play_chr_val_handle  = 0;
static bool     s_ble_ready            = false;

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

/* ==========================================================================
 *  Helpers
 * ========================================================================== */
static int16_t mv_to_angle(int mv)
{
    if (mv < 0)    mv = 0;
    if (mv > 3300) mv = 3300;
    return (int16_t)((mv * (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)) / 3300
                     + SERVO_MIN_DEGREE);
}

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

static int ble_write_no_rsp(uint16_t chr_handle, const void *data, uint16_t len)
{
    if (!s_ble_ready) return -1;
    return ble_gattc_write_no_rsp_flat(s_conn_handle, chr_handle, data, len);
}

/* ==========================================================================
 *  BLE scanning
 * ========================================================================== */
static void ble_start_scan(void)
{
    struct ble_gap_disc_params dp = {
        .passive           = 0,
        .filter_duplicates = 1,
        .itvl              = 0,
        .window            = 0,
    };
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp,
                          ble_gap_event_cb, NULL);
    if (rc != 0) ESP_LOGE(TAG, "Scan start failed: %d", rc);
    else         ESP_LOGI(TAG, "Scanning for \"%s\"...", RECEIVER_BLE_NAME);
}

/* ==========================================================================
 *  GATT service-discovery complete callback
 * ========================================================================== */
static void on_disc_complete(const struct peer *peer, int status, void *arg)
{
    if (status != 0) {
        ESP_LOGE(TAG, "Service discovery failed: %d — disconnecting", status);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    const struct peer_chr *servo_chr = peer_chr_find_uuid(
        peer, &puppet_svc_uuid.u, &servo_chr_uuid.u);
    const struct peer_chr *audio_chr = peer_chr_find_uuid(
        peer, &puppet_svc_uuid.u, &audio_chr_uuid.u);
    const struct peer_chr *play_chr  = peer_chr_find_uuid(
        peer, &puppet_svc_uuid.u, &play_chr_uuid.u);

    if (!servo_chr || !audio_chr || !play_chr) {
        ESP_LOGE(TAG, "Puppet characteristics not found — disconnecting");
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    s_servo_chr_val_handle = servo_chr->chr.val_handle;
    s_audio_chr_val_handle = audio_chr->chr.val_handle;
    s_play_chr_val_handle  = play_chr->chr.val_handle;
    s_ble_ready = true;

    ESP_LOGI(TAG, "Ready — Servo:0x%04X  Audio:0x%04X  Play:0x%04X",
             s_servo_chr_val_handle, s_audio_chr_val_handle, s_play_chr_val_handle);
}

/* ==========================================================================
 *  MTU exchange callback — initiates service discovery on completion
 * ========================================================================== */
static int on_mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                     uint16_t mtu, void *arg)
{
    if (error->status == 0) {
        ESP_LOGI(TAG, "MTU negotiated: %u bytes", mtu);
    } else {
        ESP_LOGW(TAG, "MTU exchange failed (%d) — audio may be corrupted",
                 error->status);
    }
    peer_disc_all(conn_handle, on_disc_complete, NULL);
    return 0;
}

/* ==========================================================================
 *  BLE GAP event callback
 * ========================================================================== */
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0) break;
        if (fields.name == NULL) break;
        if (fields.name_len != strlen(RECEIVER_BLE_NAME)) break;
        if (memcmp(fields.name, RECEIVER_BLE_NAME, fields.name_len) != 0) break;

        ESP_LOGI(TAG, "Found \"%s\" — connecting...", RECEIVER_BLE_NAME);
        ble_gap_disc_cancel();

        struct ble_gap_conn_params cp = {
            .scan_itvl           = 0x0010,
            .scan_window         = 0x0010,
            .itvl_min            = 6,    /* 7.5 ms */
            .itvl_max            = 12,   /* 15 ms  */
            .latency             = 0,
            .supervision_timeout = 400,  /* 4 s    */
            .min_ce_len          = 0,
            .max_ce_len          = 0,
        };
        int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                                 BLE_HS_FOREVER, &cp, ble_gap_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Connect request failed (%d) — rescanning", rc);
            ble_start_scan();
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "Connection failed (%d) — rescanning",
                     event->connect.status);
            ble_start_scan();
            break;
        }
        s_conn_handle = event->connect.conn_handle;
        peer_add(s_conn_handle);
        ESP_LOGI(TAG, "Connected (handle %u) — exchanging MTU...", s_conn_handle);
        ble_gattc_exchange_mtu(s_conn_handle, on_mtu_cb, NULL);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected (reason %d) — rescanning",
                 event->disconnect.reason);
        peer_delete(s_conn_handle);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble_ready   = false;
        ble_start_scan();
        break;

    default:
        break;
    }
    return 0;
}

/* ==========================================================================
 *  LED flash task
 * ========================================================================== */
static void led_flash_task(void *pvParameters)
{
    bool led_state = false;
    while (true) {
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        bool still_recording = s_is_recording;
        xSemaphoreGive(s_audio_mutex);

        if (!still_recording) {
            gpio_set_level(LED_RECORD_GPIO, 0);
            vTaskDelete(NULL);
            return;
        }
        led_state = !led_state;
        gpio_set_level(LED_RECORD_GPIO, led_state ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ==========================================================================
 *  Send recorded audio to receiver via BLE GATT write-without-response
 * ========================================================================== */
static void send_audio_to_receiver(const uint8_t *audio_data, uint32_t num_samples)
{
    if (num_samples == 0) {
        ESP_LOGW(TAG, "Recording empty — nothing to send");
        return;
    }
    if (!s_ble_ready) {
        ESP_LOGW(TAG, "BLE not connected — cannot send audio");
        return;
    }

    uint16_t total_frags = (uint16_t)((num_samples + AUDIO_CHUNK_BYTES - 1)
                                       / AUDIO_CHUNK_BYTES);
    ESP_LOGI(TAG, "Sending %lu samples in %u fragments", num_samples, total_frags);

    pkt_audio_start_t start_pkt = {
        .pkt_type      = PKT_TYPE_AUDIO_START,
        .total_samples = num_samples,
        .total_frags   = total_frags,
    };
    ble_write_no_rsp(s_audio_chr_val_handle, &start_pkt, sizeof(start_pkt));
    vTaskDelay(pdMS_TO_TICKS(20));

    pkt_audio_frag_t frag_pkt;
    uint32_t offset = 0;
    for (uint16_t i = 0; i < total_frags; i++) {
        uint32_t remaining = num_samples - offset;
        uint16_t chunk_len = (remaining >= AUDIO_CHUNK_BYTES)
                             ? AUDIO_CHUNK_BYTES : (uint16_t)remaining;

        frag_pkt.pkt_type    = PKT_TYPE_AUDIO_FRAG;
        frag_pkt.frag_index  = i;
        frag_pkt.total_frags = total_frags;
        frag_pkt.payload_len = chunk_len;
        memcpy(frag_pkt.audio, audio_data + offset, chunk_len);

        ble_write_no_rsp(s_audio_chr_val_handle, &frag_pkt,
                         (uint16_t)(sizeof(frag_pkt) - AUDIO_CHUNK_BYTES + chunk_len));

        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Audio transfer complete — %lu samples in %u fragments",
             num_samples, total_frags);
}

/* ==========================================================================
 *  Recording task
 *
 *  ADC1 (microphone, continuous) and ADC2 (potentiometers, oneshot) are
 *  independent units — no conflict, no task suspension required.
 * ========================================================================== */
static void recording_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Recording task started");

    ESP_ERROR_CHECK(adc_continuous_start(s_adc_mic_handle));

    uint32_t frame_bytes = (AUDIO_SAMPLE_RATE_HZ / 10) * sizeof(adc_digi_output_data_t);
    uint8_t *dma_buf = (uint8_t *)malloc(frame_bytes);
    if (!dma_buf) {
        ESP_LOGE(TAG, "DMA buffer allocation failed — aborting");
        adc_continuous_stop(s_adc_mic_handle);
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        s_is_recording = false;
        xSemaphoreGive(s_audio_mutex);
        vTaskDelete(NULL);
        return;
    }

    s_audio_samples = 0;

    while (true) {
        xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
        bool keep = s_is_recording;
        xSemaphoreGive(s_audio_mutex);
        if (!keep) break;

        if (s_audio_samples >= AUDIO_MAX_SAMPLES) {
            xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
            s_is_recording = false;
            xSemaphoreGive(s_audio_mutex);
            break;
        }

        uint32_t out_len = 0;
        esp_err_t ret = adc_continuous_read(s_adc_mic_handle, dma_buf,
                                            frame_bytes, &out_len, 10);
        if (ret == ESP_OK && out_len > 0) {
            uint32_t num_results = out_len / sizeof(adc_digi_output_data_t);
            for (uint32_t j = 0; j < num_results; j++) {
                if (s_audio_samples >= AUDIO_MAX_SAMPLES) break;
                adc_digi_output_data_t *p =
                    (adc_digi_output_data_t *)(dma_buf +
                                               j * sizeof(adc_digi_output_data_t));
                if (p->type2.channel != MIC_ADC_CHANNEL) continue;
                s_audio_buf[s_audio_samples++] = (uint8_t)(p->type2.data >> 4);
            }
        }
    }

    adc_continuous_stop(s_adc_mic_handle);
    free(dma_buf);
    ESP_LOGI(TAG, "Recording stopped — %lu samples (%.2f s)",
             s_audio_samples, (float)s_audio_samples / AUDIO_SAMPLE_RATE_HZ);

    send_audio_to_receiver(s_audio_buf, s_audio_samples);
    vTaskDelete(NULL);
}

/* ==========================================================================
 *  Button task
 * ========================================================================== */
static void button_task(void *pvParameters)
{
    bool prev_rec  = true;
    bool prev_play = true;

    while (true) {
        bool rec  = (gpio_get_level(BTN_RECORD_GPIO) == 1);
        bool play = (gpio_get_level(BTN_PLAY_GPIO)   == 1);

        if (prev_rec && !rec) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (gpio_get_level(BTN_RECORD_GPIO) == 0) {
                xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                bool currently = s_is_recording;
                xSemaphoreGive(s_audio_mutex);

                if (!currently) {
                    s_audio_samples = 0;
                    xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                    s_is_recording = true;
                    xSemaphoreGive(s_audio_mutex);
                    ESP_LOGI(TAG, "Recording STARTED");
                    xTaskCreatePinnedToCore(led_flash_task, "led", 2048, NULL, 3, NULL, 1);
                    xTaskCreatePinnedToCore(recording_task, "rec", 8192, NULL, 6, NULL, 1);
                } else {
                    xSemaphoreTake(s_audio_mutex, portMAX_DELAY);
                    s_is_recording = false;
                    xSemaphoreGive(s_audio_mutex);
                    ESP_LOGI(TAG, "Recording STOPPED");
                }
            }
        }

        if (prev_play && !play) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (gpio_get_level(BTN_PLAY_GPIO) == 0) {
                pkt_play_t p = { .pkt_type = PKT_TYPE_AUDIO_PLAY };
                if (ble_write_no_rsp(s_play_chr_val_handle, &p, sizeof(p)) == 0) {
                    ESP_LOGI(TAG, "Play command sent");
                } else {
                    ESP_LOGW(TAG, "Play command failed — not connected");
                }
            }
        }

        prev_rec  = rec;
        prev_play = play;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ==========================================================================
 *  Servo task — reads ADC2 potentiometers and sends angle commands at 20 Hz
 * ========================================================================== */
static void servo_task(void *pvParameters)
{
    pkt_servo_t cmd = { .pkt_type = PKT_TYPE_SERVO };
    ESP_LOGI(TAG, "Servo task started");

    while (true) {
        int mv1 = pot_read_mv(POT1_ADC_CHANNEL, s_cali_pot1);
        int mv2 = pot_read_mv(POT2_ADC_CHANNEL, s_cali_pot2);
        cmd.servo1_angle = mv_to_angle(mv1);
        cmd.servo2_angle = mv_to_angle(mv2);
        if (s_ble_ready) {
            ble_write_no_rsp(s_servo_chr_val_handle, &cmd, sizeof(cmd));
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_SEND_INTERVAL_MS));
    }
}

/* ==========================================================================
 *  NimBLE stack callbacks
 * ========================================================================== */
static void ble_on_sync(void)
{
    ESP_ERROR_CHECK(ble_hs_util_ensure_addr(0));
    uint8_t addr[6];
    ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL);
    ESP_LOGI(TAG, "BLE ready. MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    ble_start_scan();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset: %d", reason);
}

static void nimble_host_task(void *pvParameters)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ==========================================================================
 *  ADC initialisations
 * ========================================================================== */
static void pot_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_2,       /* ADC2 — free because BLE != WiFi radio */
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_pot_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_pot_handle, POT1_ADC_CHANNEL, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_pot_handle, POT2_ADC_CHANNEL, &ch_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_2,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_pot1) != ESP_OK)
        s_cali_pot1 = NULL;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_pot2) != ESP_OK)
        s_cali_pot2 = NULL;

    ESP_LOGI(TAG, "Potentiometer ADC2 initialised (GPIO %d / %d)", POT1_GPIO, POT2_GPIO);
}

static void mic_adc_init(void)
{
    uint32_t buf_size = (AUDIO_SAMPLE_RATE_HZ / 10) * sizeof(adc_digi_output_data_t);

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = buf_size * 4,
        .conv_frame_size    = buf_size,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &s_adc_mic_handle));

    adc_digi_pattern_config_t pat = {
        .atten     = ADC_ATTEN_DB_12,
        .channel   = MIC_ADC_CHANNEL,
        .unit      = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t cont_cfg = {
        .sample_freq_hz = AUDIO_SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num    = 1,
        .adc_pattern    = &pat,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc_mic_handle, &cont_cfg));
    ESP_LOGI(TAG, "Microphone ADC1 (continuous DMA) at %d Hz", AUDIO_SAMPLE_RATE_HZ);
}

static void gpio_setup(void)
{
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_RECORD_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));
    gpio_set_level(LED_RECORD_GPIO, 0);

    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BTN_RECORD_GPIO) | (1ULL << BTN_PLAY_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));
}

/* ==========================================================================
 *  app_main
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "  ESP32-S3 Puppet Controller");
    ESP_LOGI(TAG, "  BLE Central | ESP-IDF v6.0.1");
    ESP_LOGI(TAG, "================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_audio_buf = (uint8_t *)heap_caps_malloc(AUDIO_MAX_SAMPLES,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_audio_buf) s_audio_buf = (uint8_t *)malloc(AUDIO_MAX_SAMPLES);
    if (!s_audio_buf) {
        ESP_LOGE(TAG, "FATAL: Cannot allocate %d byte audio buffer", AUDIO_MAX_SAMPLES);
        return;
    }

    s_audio_mutex = xSemaphoreCreateMutex();

    gpio_setup();
    pot_adc_init();
    mic_adc_init();

    ESP_ERROR_CHECK(nimble_port_init());

    /* Set preferred MTU before any connection.
     * Audio fragments are 207 bytes; the default BLE MTU (23 bytes) would
     * truncate every fragment → corrupted audio → noise on the receiver.    */
    ble_att_set_preferred_mtu(BLE_PREFERRED_MTU);

    peer_init(PEER_MAX_PEERS, PEER_MAX_SVCS, PEER_MAX_CHRS, PEER_MAX_DSCS);

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    nimble_port_freertos_init(nimble_host_task);

    xTaskCreatePinnedToCore(servo_task,  "servo",   4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "buttons", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Controller ready — scanning for receiver...");
    ESP_LOGI(TAG, "  Pots → GPIO %d / %d (ADC2)", POT1_GPIO, POT2_GPIO);
    ESP_LOGI(TAG, "  Mic  → GPIO %d (ADC1)", MIC_GPIO);
}
