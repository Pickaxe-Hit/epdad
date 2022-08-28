#include "epd_board.h"

#include "pca9557.h"

#include "esp_err.h"

esp_err_t epd_panel_on(void) {
    esp_err_t err;
    ESP_ERROR_CHECK(pca9557_set_config(I2C_NUM_0, EPD_EN_PORT, 0));
    err = pca9557_set_value(I2C_NUM_0, EPD_EN_PORT, 1);
    return err;
}

esp_err_t epd_panel_off(void) {
    esp_err_t err;
    ESP_ERROR_CHECK(pca9557_set_config(I2C_NUM_0, EPD_EN_PORT, 0));
    err = pca9557_set_value(I2C_NUM_0, EPD_EN_PORT, 0);
    return err;
}

esp_err_t epd_power_on(void) {
    esp_err_t err;
    ESP_ERROR_CHECK(pca9557_set_config(I2C_NUM_0, EPD_DCEN_PORT, 0));
    err = pca9557_set_value(I2C_NUM_0, EPD_DCEN_PORT, 1);
    return err;
}

esp_err_t epd_power_off(void) {
    esp_err_t err;
    ESP_ERROR_CHECK(pca9557_set_config(I2C_NUM_0, EPD_DCEN_PORT, 0));
    err = pca9557_set_value(I2C_NUM_0, EPD_DCEN_PORT, 0);
    return err;
}
esp_err_t i2c_conv_on(void) {
    esp_err_t err;
    ESP_ERROR_CHECK(pca9557_set_config(I2C_NUM_0, EPD_DCEN_PORT, 0));
    err = pca9557_set_value(I2C_NUM_0, EPD_DCEN_PORT, 1);
    return err;
}

esp_err_t i2c_conv_off(void) {
    esp_err_t err;
    ESP_ERROR_CHECK(pca9557_set_config(I2C_NUM_0, TXS0104_EN_PORT, 0));
    err = pca9557_set_value(I2C_NUM_0, TXS0104_EN_PORT, 0);
    return err;
}