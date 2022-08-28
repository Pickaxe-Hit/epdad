#include "epd_driver.h"

#include "epd_board.h"

#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

uint8_t *epd_gram;

void epd_gram_init(void) {
    uint32_t gram_size = EPD_H_RES * EPD_V_RES / 4;
    epd_gram = heap_caps_calloc(gram_size, sizeof(uint8_t), MALLOC_CAP_SPIRAM);
}

void epd_send_data(void) {
    epd_gram_init();
    REG_WRITE(GPIO_ENABLE_W1TS_REG, ((1 << EPD_XCLK_GPIO) |
                                     (1 << EPD_LD_GPIO) |
                                     (1 << EPD_YCLK_GPIO) |
                                     (1 << EPD_YDIO_GPIO) |
                                     (1 << EPD_YOE_GPIO) |
                                     (1 << EPD_XDIO_GPIO)));
    esp_rom_delay_us(2);

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_LD_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_LD_GPIO) | (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_XCLK_GPIO));

    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = EPD_XDIO_GPIO,
        .wr_gpio_num = EPD_XCLK_GPIO,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = {EPD_D0_GPIO,
                           EPD_D1_GPIO,
                           EPD_D2_GPIO,
                           EPD_D3_GPIO,
                           EPD_D4_GPIO,
                           EPD_D5_GPIO,
                           EPD_D6_GPIO,
                           EPD_D7_GPIO},
        .bus_width = 8,
        .max_transfer_bytes = 100,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = -1,
        .pclk_hz = 8000000,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 1,
            .dc_dummy_level = 0,
            .dc_data_level = 0,
        },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_new_i80_bus(&bus_config, &i80_bus);
    esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle);

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_YDIO_GPIO));
    for (int i = 0; i < 768; i++) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_YOE_GPIO));
        esp_rom_delay_us(1);
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_YCLK_GPIO));
        esp_rom_delay_us(1);
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_YCLK_GPIO) | (1 << EPD_YOE_GPIO));
        if (i == 0) {
            REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_YDIO_GPIO));
        };
        esp_lcd_panel_io_tx_param(io_handle, 0x00, (epd_gram + i * EPD_H_RES / 4), EPD_H_RES / 4);
    }
    esp_lcd_panel_io_del(io_handle);
    esp_lcd_del_i80_bus(i80_bus);
}