# Chirp-catcher build (PSRAM clips + Wi-Fi SoftAP download page).
# Older firmware variants live in archive/ — to build one, copy it here
# over main.c (archive/main.c also needs: sdmmc fatfs vfs + esp_tinyusb).
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2s driver esp_timer esp_psram
                             nvs_flash esp_netif esp_event esp_wifi esp_http_server)
