idf_component_register(
    SRCS "nmea2000_gsm.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_netif esp_event esp_http_server esp_http_client esp-tls nvs_flash lwip
)
