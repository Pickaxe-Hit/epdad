#include "esp_err.h"

#define EPD_XCLK_GPIO 14
#define EPD_YCLK_GPIO 18

#define EPD_XDIO_GPIO 13
#define EPD_YDIO_GPIO 21

#define EPD_LD_GPIO 4

#define EPD_YOE_GPIO 17

#define EPD_D0_GPIO 5
#define EPD_D1_GPIO 6
#define EPD_D2_GPIO 7
#define EPD_D3_GPIO 8
#define EPD_D4_GPIO 9
#define EPD_D5_GPIO 10
#define EPD_D6_GPIO 11
#define EPD_D7_GPIO 12

#define EPD_VCOM0_GPIO 15
#define EPD_VCOM1_GPIO 16

#define I2C_EPD_SCL_IO 39
#define I2C_EPD_SDA_IO 40

#define EPD_EN_PORT 6
#define EPD_DCEN_PORT 5
#define TXS0104_EN_PORT 7

#define I2C_PORT I2C_NUM_0

esp_err_t epd_panel_on(void);

esp_err_t epd_panel_off(void);

esp_err_t epd_power_on(void);

esp_err_t epd_power_off(void);

esp_err_t i2c_conv_on(void);

esp_err_t i2c_conv_off(void);

esp_err_t epd_board_reset(void);
