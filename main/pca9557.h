#include "esp_err.h"
#include "i2c_master.h"

#define PCA9557_ADDR 0x18

#define PCA9557_IN_REG 0x0
#define PCA9557_OUT_REG 0x1
#define PCA9557_INV_REG 0x2
#define PCA9557_CONF_REG 0x3

esp_err_t pca9557_set_config(i2c_port_t i2c_num,
                             uint8_t pin_num,
                             uint8_t input);

esp_err_t pca9557_set_inversion(i2c_port_t i2c_num,
                                uint8_t pin_num,
                                uint8_t enable);

esp_err_t pca9557_set_value(i2c_port_t i2c_num,
                            uint8_t pin_num,
                            uint8_t value);

uint8_t pca9557_read_input(i2c_port_t i2c_num,
                           int high_port);