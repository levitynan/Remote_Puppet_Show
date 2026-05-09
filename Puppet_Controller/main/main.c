/*
 * =============================================================================
 *  CONTROLLER FIRMWARE — ESP32-S3-Nano Development Board
 *  Framework : ESP-IDF v6.0.1
 *  Purpose   : Read two potentiometers (or any analogue input) to determine
 *              desired servo angles, then broadcast the angles to the receiver
 *              board via ESP-NOW.
 *
 *  Hardware wiring (adjust GPIO numbers to match your board layout):
 *    Potentiometer 1 wiper → GPIO 1  (ADC1 channel 0)
 *    Potentiometer 2 wiper → GPIO 2  (ADC1 channel 1)
 *    Both pot rails: one end → 3.3 V, other end → GND
 *
 *  Before flashing this firmware:
 *    1. Flash the receiver board first.
 *    2. Open its serial monitor and note the MAC address it prints.
 *    3. Replace the placeholder bytes in RECEIVER_MAC_ADDR below with the
 *       actual MAC address of your receiver board.
 * =============================================================================
 */

/* -------------------------------------------------------------------------- */
/*  Standard & FreeRTOS includes                                              */
/* -------------------------------------------------------------------------- */
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

/* ADC (oneshot driver — recommended in IDF v5+) */
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* -------------------------------------------------------------------------- */
/*  Logging tag                                                               */
/* -------------------------------------------------------------------------- */
static const char *TAG = "CONTROLLER";

/* -------------------------------------------------------------------------- */
/*  *** IMPORTANT: Replace these bytes with your receiver board's MAC ***     */
/*                                                                             */
/*  Run the receiver firmware first, check its serial log for a line like:    */
/*    Receiver MAC address: AA:BB:CC:DD:EE:FF                                 */
/*  Then fill in those 6 bytes below.                                         */
/* -------------------------------------------------------------------------- */
static const uint8_t RECEIVER_MAC_ADDR[6] = {0xDC, 0xB4, 0xD9, 0x25, 0x79, 0xDC};

/* -------------------------------------------------------------------------- */
/*  ADC GPIO pins for the two potentiometers                                   */
/* -------------------------------------------------------------------------- */
#define POT1_ADC_CHANNEL    ADC_CHANNEL_0   /* GPIO 1 on ESP32-S3  */
#define POT2_ADC_CHANNEL    ADC_CHANNEL_1   /* GPIO 2 on ESP32-S3  */
#define ADC_UNIT            ADC_UNIT_1      /* ADC unit 1 (GPIO 1–10 on ESP32-S3) */

/* -------------------------------------------------------------------------- */
/*  Servo angle range — must match the receiver constants                      */
/* -------------------------------------------------------------------------- */
#define SERVO_MIN_DEGREE    (-90)
#define SERVO_MAX_DEGREE    90

/* -------------------------------------------------------------------------- */
/*  How often  to sample and send a command (milliseconds)                      */
/*  50 ms = 20 Hz update rate — smooth for most servo applications.           */
/* -------------------------------------------------------------------------- */
#define SEND_INTERVAL_MS    50

/* -------------------------------------------------------------------------- */
/*  Shared command structure                                                   */
/*  *** This struct MUST be byte-for-byte identical in receiver/main.c ***    */
/* -------------------------------------------------------------------------- */
typedef struct {
    int16_t servo1_angle;   /* Target angle for servo 1, range –90 to +90 degrees */
    int16_t servo2_angle;   /* Target angle for servo 2, range –90 to +90 degrees */
} servo_command_t;

/* ADC handles — initialised once, used throughout the task */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t         s_cali_handle_ch0 = NULL;
static adc_cali_handle_t         s_cali_handle_ch1 = NULL;

/* ========================================================================== */
/*  ESP-NOW send callback                                                      */
/*                                                                             */
/*  Called by the Wi-Fi driver after each esp_now_send() completes.           */
/*  Use it for debugging — it tells you whether the packet was acknowledged.  */
/*  NOTE: the receiver does NOT send an ACK at the application layer; the     */
/*  "success" status here means the MAC layer received an ACK from the peer.  */
/* ========================================================================== */
// ✅ NEW — v6 signature using esp_now_send_info_t
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        /* MAC address is now accessed via tx_info->des_addr (destination address) */
        ESP_LOGD(TAG, "Packet delivered to " MACSTR, MAC2STR(tx_info->des_addr));
    } else {
        ESP_LOGW(TAG, "Packet delivery FAILED to " MACSTR, MAC2STR(tx_info->des_addr));
    }
}

/* ========================================================================== */
/*  Wi-Fi + ESP-NOW Initialisation                                             */
/* ========================================================================== */
static void wifi_espnow_init(void)
{
    /* NVS flash — required by Wi-Fi driver */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Networking stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Wi-Fi — station mode, no AP connection required for ESP-NOW */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Print this board's MAC address for reference */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "Controller MAC address: " MACSTR, MAC2STR(mac));

    /* ESP-NOW */
    ESP_ERROR_CHECK(esp_now_init());

    /* Register send status callback */
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    /* ------------------------------------------------------------------ */
    /*  Register the receiver as a peer                                    */
    /*                                                                     */
    /*  A peer must be registered before esp_now_send() can deliver        */
    /*  packets to it. channel = 0 means "use the current Wi-Fi channel". */
    /*  encrypt = false keeps setup simple; enable and provide lmk[] if  */
    /*  you need AES-128 payload encryption.                              */
    /* ------------------------------------------------------------------ */
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, RECEIVER_MAC_ADDR, 6);
    peer_info.channel = 0;       /* 0 = use current Wi-Fi channel */
    peer_info.encrypt = false;   /* No payload encryption          */
    peer_info.ifidx   = WIFI_IF_STA;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ESP-NOW peer — check RECEIVER_MAC_ADDR");
    } else {
        ESP_LOGI(TAG, "Receiver peer registered: " MACSTR, MAC2STR(RECEIVER_MAC_ADDR));
    }
}

/* ========================================================================== */
/*  ADC Initialisation                                                         */
/*                                                                             */
/*  The oneshot ADC driver (IDF v5+) is used rather than the deprecated       */
/*  continuous driver. Calibration is applied when available to convert raw   */
/*  ADC counts into millivolts for a more accurate angle mapping.             */
/* ========================================================================== */
static void adc_init(void)
{
    /* Create an ADC oneshot unit handle for ADC_UNIT_1 */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id  = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE, /* Normal (not ULP) operation */
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_handle));

    /* Configure both channels: 12-bit resolution, 11 dB attenuation       */
    /* 11 dB attenuation gives an effective input range of ~0–3.1 V on     */
    /* ESP32-S3, suitable for a 0–3.3 V potentiometer divider.             */
    adc_oneshot_chan_cfg_t chan_config = {
        .atten    = ADC_ATTEN_DB_12,    /* Full-range attenuation (0–3.1 V) */
        .bitwidth = ADC_BITWIDTH_DEFAULT, /* 12-bit (4096 counts)           */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, POT1_ADC_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, POT2_ADC_CHANNEL, &chan_config));

    /* ------------------------------------------------------------------ */
    /*  Calibration — uses eFuse-stored curve fitting data when available  */
    /*  Falls back gracefully if calibration data is not present.         */
    /* ------------------------------------------------------------------ */
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    /* Channel 0 calibration */
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle_ch0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC ch0 calibration not available — using raw counts");
        s_cali_handle_ch0 = NULL;
    }

    /* Channel 1 calibration (reuse same config — same unit/atten/bitwidth) */
    err = adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle_ch1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC ch1 calibration not available — using raw counts");
        s_cali_handle_ch1 = NULL;
    }

    ESP_LOGI(TAG, "ADC initialised on channels %d and %d", POT1_ADC_CHANNEL, POT2_ADC_CHANNEL);
}

/* ========================================================================== */
/*  Helper: read ADC channel and return voltage in millivolts                  */
/*                                                                             */
/*  If calibration is available, converts raw counts → mV using the           */
/*  eFuse-calibrated curve. Otherwise returns raw counts scaled to mV.        */
/* ========================================================================== */
static int adc_read_mv(adc_channel_t channel, adc_cali_handle_t cali_handle)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, channel, &raw));

    if (cali_handle != NULL) {
        /* Convert raw ADC count to calibrated millivolts */
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv));
        return voltage_mv;
    }

    /* No calibration: scale raw 12-bit count (0–4095) to 0–3300 mV linearly */
    return (raw * 3300) / 4095;
}

/* ========================================================================== */
/*  Helper: map a millivolt value (0–3300 mV) to a servo angle (–90 to +90°)  */
/* ========================================================================== */
static int16_t mv_to_angle(int voltage_mv)
{
    /* Clamp voltage to valid ADC range before mapping */
    if (voltage_mv < 0)    voltage_mv = 0;
    if (voltage_mv > 3300) voltage_mv = 3300;

    /* Linear map: 0 mV → –90°,  1650 mV → 0°,  3300 mV → +90° */
    int16_t angle = (int16_t)(
        (voltage_mv * (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)) / 3300
        + SERVO_MIN_DEGREE
    );

    return angle;
}

/* ========================================================================== */
/*  Controller task — samples pots, builds command, and sends via ESP-NOW     */
/* ========================================================================== */
static void controller_task(void *pvParameters)
{
    servo_command_t cmd;

    ESP_LOGI(TAG, "Controller task started — sending at %d ms intervals", SEND_INTERVAL_MS);

    while (1) {
        /* ---------------------------------------------------------------- */
        /*  1. Read potentiometer voltages                                  */
        /* ---------------------------------------------------------------- */
        int mv1 = adc_read_mv(POT1_ADC_CHANNEL, s_cali_handle_ch0);
        int mv2 = adc_read_mv(POT2_ADC_CHANNEL, s_cali_handle_ch1);

        /* ---------------------------------------------------------------- */
        /*  2. Map voltage → angle                                          */
        /* ---------------------------------------------------------------- */
        cmd.servo1_angle = mv_to_angle(mv1);
        cmd.servo2_angle = mv_to_angle(mv2);

        ESP_LOGI(TAG, "Pot1=%d mV → %d°  |  Pot2=%d mV → %d°",
                 mv1, cmd.servo1_angle,
                 mv2, cmd.servo2_angle);

        /* ---------------------------------------------------------------- */
        /*  3. Send via ESP-NOW to the registered receiver peer             */
        /*                                                                  */
        /*  esp_now_send() is non-blocking; the espnow_send_cb() callback  */
        /*  will fire asynchronously when the MAC-layer result is known.    */
        /* ---------------------------------------------------------------- */
        esp_err_t result = esp_now_send(
            RECEIVER_MAC_ADDR,
            (const uint8_t *)&cmd,
            sizeof(servo_command_t)
        );

        if (result != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_send() failed: %s", esp_err_to_name(result));
        }

        /* ---------------------------------------------------------------- */
        /*  4. Wait before the next sample                                  */
        /* ---------------------------------------------------------------- */
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }

    /* Should never reach here */
    vTaskDelete(NULL);
}

/* ========================================================================== */
/*  app_main — entry point                                                     */
/* ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "  ESP32-S3 Servo Controller");
    ESP_LOGI(TAG, "  ESP-IDF v6.0.1");
    ESP_LOGI(TAG, "==============================");

    /* Initialise ADC for potentiometer inputs */
    adc_init();

    /* Initialise Wi-Fi and ESP-NOW, register receiver as a peer */
    wifi_espnow_init();

    /* Start the controller task on core 0, priority 5 */
    xTaskCreatePinnedToCore(
        controller_task,    /* Task function                   */
        "controller",       /* Task name                       */
        4096,               /* Stack size in bytes             */
        NULL,               /* Task parameter (not used)       */
        5,                  /* Priority                        */
        NULL,               /* Task handle (not stored)        */
        0                   /* CPU core 0                      */
    );

    ESP_LOGI(TAG, "Initialisation complete. Controller is active.");
}