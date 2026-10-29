idf_component_register(
        SRCS "esp32_firmware.c"
        INCLUDE_DIRS "."
        REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_driver_gpio
)
