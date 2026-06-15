idf_component_register(SRCS "main.c" "sensors.c" "wifi_config.c" "ota.c" "discovery.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_wifi esp_http_server mdns esp_timer app_update esp_app_format can_common )

target_compile_definitions(${COMPONENT_LIB} PRIVATE MODULE_TYPE="australis")
