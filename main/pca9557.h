#include "driver/i2c.h"
#include "esp_err.h"

#define PCA9557_ADDR 0x18

#define PCA9557_IN_REG 0x0
#define PCA9557_OUT_REG 0x1
#define PCA9557_INV_REG 0x2
#define PCA9557_CONF_REG 0x3

esp_err_t pca9557_set_config(i2c_port_t i2c_num,
                             uint8_t pin_num,
                             uint8_t input_en);

esp_err_t pca9557_set_inversion(i2c_port_t i2c_num,
                                uint8_t pin_num,
                                uint8_t enable);

esp_err_t pca9557_set_level(i2c_port_t i2c_num,
                            uint8_t pin_num,
                            uint8_t level);

uint8_t pca9557_read_input(i2c_port_t i2c_num, uint8_t pin_num);