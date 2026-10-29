
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <inttypes.h>
#include "freertos/queue.h"
// WLAN-Zugangsdaten
#define WIFI_SSID "ESP32-CSI-NET"
#define WIFI_PASS "CSI-Roboter2026"

// LED für WLAN-Verbindungsstatus
#define WIFI_LED_PIN GPIO_NUM_26

#define WIFI_CONNECTED_BIT BIT0

#define CSI_MAX_BYTES 768
#define CSI_QUEUE_LENGTH 8

typedef struct {
    int8_t rssi;
    uint16_t length;
    bool first_word_invalid;

    uint8_t mac[6];
    uint8_t channel;
    uint8_t sig_mode;
    uint8_t cwb;
    uint8_t stbc;
    uint8_t secondary_channel;

    uint32_t timestamp_us;

    int8_t data[CSI_MAX_BYTES];
} csi_packet_t;

static QueueHandle_t csi_queue = NULL;
static uint32_t dropped_packets = 0;


static EventGroupHandle_t wifi_event_group;
static const char *TAG = "ESP32_CSI";

// ========================================
// LED-FUNKTIONEN
// ========================================

// Initialisiert einen übergebenen GPIO als Ausgang
static void init_led(gpio_num_t pin)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&config));

    // LED standardmäßig ausschalten
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

// Schaltet eine LED am angegebenen Pin
static void set_led(gpio_num_t pin, bool state)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, state ? 1 : 0));
}

// ========================================
// WLAN-EVENT-HANDLER
// ========================================

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        set_led(WIFI_LED_PIN, false);
        esp_wifi_connect();
    }

    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

        // WLAN getrennt: LED ausschalten
        set_led(WIFI_LED_PIN, false);

        xEventGroupClearBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );

        ESP_LOGW(TAG, "WLAN getrennt. Verbinde erneut...");
        esp_wifi_connect();
    }

    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "IP-Adresse: " IPSTR,
                 IP2STR(&event->ip_info.ip));

        // WLAN verbunden: LED einschalten
        set_led(WIFI_LED_PIN, true);

        xEventGroupSetBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );

        ESP_LOGI(TAG, "WLAN verbunden! LED an GPIO26 leuchtet.");
    }
}

// ========================================
// CSI-CALLBACK
// ========================================





static void csi_callback(void *ctx, wifi_csi_info_t *data)
{
    if (data == NULL || data->buf == NULL ||
        csi_queue == NULL) {
        return;
        }

    if (data->len > CSI_MAX_BYTES) {
        dropped_packets++;
        return;
    }

    csi_packet_t packet = {0};

    packet.rssi = data->rx_ctrl.rssi;
    packet.length = data->len;

    packet.first_word_invalid =
        data->first_word_invalid;

    memcpy(packet.mac, data->mac, 6);

    packet.channel = data->rx_ctrl.channel;
    packet.sig_mode = data->rx_ctrl.sig_mode;
    packet.cwb = data->rx_ctrl.cwb;
    packet.stbc = data->rx_ctrl.stbc;

    packet.secondary_channel =
        data->rx_ctrl.secondary_channel;

    packet.timestamp_us =
        data->rx_ctrl.timestamp;

    memcpy(packet.data, data->buf, data->len);

    if (xQueueSend(csi_queue, &packet, 0) != pdTRUE) {
        dropped_packets++;
    }
}

static void csi_json_task(void *arg)
{
    csi_packet_t packet;

    while (true) {

        if (xQueueReceive(
                csi_queue,
                &packet,
                portMAX_DELAY) != pdTRUE) {
            continue;
                }

        // JSON-Objekt beginnen
        printf("{");

        printf("\"timestamp_us\":%" PRIu32 ",",
               packet.timestamp_us);

        printf("\"rssi\":%d,",
               packet.rssi);

        printf("\"channel\":%u,",
               packet.channel);

        printf("\"source_mac\":"
               "\"%02x:%02x:%02x:%02x:%02x:%02x\",",
               packet.mac[0], packet.mac[1],
               packet.mac[2], packet.mac[3],
               packet.mac[4], packet.mac[5]);

        printf("\"sig_mode\":%u,",
               packet.sig_mode);

        printf("\"cwb\":%u,",
               packet.cwb);

        printf("\"stbc\":%u,",
               packet.stbc);

        printf("\"secondary_channel\":%u,",
               packet.secondary_channel);

        printf("\"length\":%u,",
               packet.length);

        printf("\"first_word_invalid\":%s,",
               packet.first_word_invalid
                   ? "true" : "false");

        printf("\"csi_data\":[");

        // Alle CSI-Rohwerte ausgeben
        for (int i = 0; i < packet.length; i++) {

            if (i > 0) {
                printf(",");
            }

            printf("%d", packet.data[i]);
        }

        printf("]}\n");

        fflush(stdout);
    }
}




// ========================================
// WLAN INITIALISIEREN
// ========================================

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    configASSERT(wifi_event_group != NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        NULL
    ));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_set_config(
        WIFI_IF_STA,
        &wifi_config
    ));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Verbinde mit WLAN...");

    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    ESP_LOGI(TAG, "WLAN verbunden!");
}

// ========================================
// CSI INITIALISIEREN
// ========================================

static void csi_init(void)
{
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = false,
        .ltf_merge_en = false,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0
    };

    ESP_ERROR_CHECK(
        esp_wifi_set_csi_rx_cb(csi_callback, NULL)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_csi_config(&csi_config)
    );

    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI aktiviert!");
}


static void udp_client_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (sock < 0) {
        ESP_LOGE(TAG, "UDP-Socket konnte nicht erstellt werden");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(5001)
    };

    inet_pton(
        AF_INET,
        "192.168.4.1",
        &server_addr.sin_addr
    );

    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = 50000
    };

    setsockopt(
        sock,
        SOL_SOCKET,
        SO_RCVTIMEO,
        &timeout,
        sizeof(timeout)
    );

    char response[64];

    while (true) {
        const char message[] = "CSI_REQUEST";

        sendto(
            sock, message, sizeof(message) - 1, 0,
            (struct sockaddr *)&server_addr,
            sizeof(server_addr)
        );

        recvfrom(
            sock,
            response,
            sizeof(response),
            0,
            NULL,
            NULL
        );

        // Etwa zehn Anfragen pro Sekunde
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ========================================
// HAUPTPROGRAMM
// ========================================

void app_main(void)
{
    // Zuerst LED initialisieren
    init_led(WIFI_LED_PIN);

    // Flash-Speicher initialisieren
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


    wifi_init();

    // Queue für CSI-Pakete erstellen
    csi_queue = xQueueCreate(
        CSI_QUEUE_LENGTH,
        sizeof(csi_packet_t)
    );

    configASSERT(csi_queue != NULL);

    // Task zur JSON-Ausgabe starten
    BaseType_t result = xTaskCreate(
        csi_json_task,
        "csi_json_task",
        4096,
        NULL,
        5,
        NULL
    );

    configASSERT(result == pdPASS);

    // CSI erst danach aktivieren
    csi_init();

    xTaskCreate(
    udp_client_task,
    "udp_client",
    4096,
    NULL,
    5,
    NULL
    );

    while (true) {
        //ESP_LOGI(TAG, "ESP32 läuft, WLAN verbunden!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
