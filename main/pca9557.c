#include "pca9557.h"

#include "i2c_master.h"

#include "driver/i2c.h"
#include "esp_log.h"

uint8_t pca9557_out_reg = 0x00;
uint8_t pca9557_inv_reg = 0x00;
uint8_t pca9557_conf_reg = 0xff;

static esp_err_t pca9557_write_single(i2c_port_t i2c_num, int reg, uint8_t value) {
    uint8_t data[1] = {value};
    return i2c_master_write_slave(i2c_num, PCA9557_ADDR, reg, data, sizeof(data));
}

esp_err_t pca9557_set_config(i2c_port_t i2c_num, uint8_t pin_num, uint8_t input_en) {
    if (input_en) {
        *(&pca9557_conf_reg) |= (1 << pin_num);
    } else {
        *(&pca9557_conf_reg) &= ~(1 << pin_num);
    }
    return pca9557_write_single(i2c_num, PCA9557_CONF_REG, *(&pca9557_conf_reg));
}

esp_err_t pca9557_set_inversion(i2c_port_t i2c_num, uint8_t pin_num, uint8_t enable) {
    if (enable) {
        *(&pca9557_inv_reg) |= (1 << pin_num);
    } else {
        *(&pca9557_inv_reg) &= ~(1 << pin_num);
    }
    return pca9557_write_single(i2c_num, PCA9557_INV_REG, *(&pca9557_inv_reg));
}

esp_err_t pca9557_set_level(i2c_port_t i2c_num, uint8_t pin_num, uint8_t level) {
    if (level) {
        *(&pca9557_out_reg) |= (1 << pin_num);
    } else {
        *(&pca9557_out_reg) &= ~(1 << pin_num);
    }
    return pca9557_write_single(i2c_num, PCA9557_OUT_REG, *(&pca9557_out_reg));
}

uint8_t pca9557_read_input(i2c_port_t i2c_num, uint8_t pin_num) {
    esp_err_t err;
    uint8_t r_data[1];

    err = i2c_master_read_slave(i2c_num, PCA9557_ADDR, r_data, 1, PCA9557_IN_REG);
    if (err != ESP_OK) {
        ESP_LOGE("PCA9557", "%s failed", __func__);
        return 0;
    }

    r_data[0] = ((r_data[0] & (1 << pin_num)) >> pin_num);

    return r_data[0];
}