idf_component_register(SRCS "nmea2000_ap.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp_http_server nvs_flash esp_wifi esp_netif driver)