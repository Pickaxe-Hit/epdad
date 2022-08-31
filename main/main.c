#include "epd_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_master.h"
#include "pca9557.h"
#include <stdio.h>

void app_main(void) {
    ESP_ERROR_CHECK(epd_panel_init());
    ESP_ERROR_CHECK(epd_send_data());
    ESP_ERROR_CHECK(epd_panel_del());
}