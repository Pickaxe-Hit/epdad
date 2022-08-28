#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_FREQ_HZ 400000

esp_err_t i2c_master_init(void);

esp_err_t i2c_master_read_slave(i2c_port_t i2c_num,
                                uint8_t i2c_slave_addr,
                                uint8_t *data_rd,
                                size_t size,
                                int reg);

esp_err_t i2c_master_write_slave(i2c_port_t i2c_num,
                                 uint8_t i2c_slave_addr,
                                 uint8_t ctrl,
                                 uint8_t *data_wr,
                                 size_t size);