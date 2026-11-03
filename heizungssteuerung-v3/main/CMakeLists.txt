idf_component_register(SRCS "app_main.c" "touch_display.c" "sensors.c" "outputs.c" "heating.c"
 "mixer.c" "boiler.c" "buffer.c" "pv.c" "mqtt.c" "ntp.c" "config.c"
 "config_rules.c" "ethernet.c" "ds18b20.c" "web.c" "service_config.c" "web_auth.c" "stove.c" "stove_rules.c" "gas.c" "gas_rules.c" "backup.c" "ota.c" "ota_rules.c" "mqtt_topics.c" "rest_heat.c" "rest_heat_rules.c"
 INCLUDE_DIRS "." REQUIRES esp_lcd_ek79007 esp_lcd_touch_gt911 lvgl esp_psram esp_driver_i2c esp_hw_support app_update esp_driver_gpio esp_eth esp_netif esp_event mqtt
 nvs_flash json esp_timer esp_system lwip esp_rom esp-tls esp_http_server esp_app_format mbedtls
 EMBED_TXTFILES "web/index.html" "web/favicon.svg")
