set(vendor "${CMAKE_CURRENT_LIST_DIR}/../../.local/display-dependencies/esp-iot-solution/components/display/lcd/esp_lcd_ek79007")
if(NOT EXISTS "${vendor}/esp_lcd_ek79007.c")
    message(FATAL_ERROR "Displayquellen fehlen. Bitte bash scripts/setup-display-deps.sh ausfuehren.")
endif()
idf_component_register(SRCS "${vendor}/esp_lcd_ek79007.c" INCLUDE_DIRS "${vendor}/include" REQUIRES esp_lcd driver)
# Version der im Setup-Skript festgelegten offiziellen Quelle.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_LCD_EK79007_VER_MAJOR=2 ESP_LCD_EK79007_VER_MINOR=0 ESP_LCD_EK79007_VER_PATCH=2)
