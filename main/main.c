#include "epd_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_master.h"
#include "pca9557.h"
#include <stdio.h>

void app_main(void) {
    i2c_master_init();
    pca9557_set_config(I2C_NUM_0, 1, false);
    pca9557_set_config(I2C_NUM_0, 7, true);
    pca9557_set_value(I2C_NUM_0, 1, 1);
    pca9557_set_value(I2C_NUM_0, 1, 0);
    pca9557_read_input(I2C_NUM_0, 7);
}