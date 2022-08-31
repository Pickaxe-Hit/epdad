#include "epd_driver.h"

#include "epd_board.h"
#include "i2c_master.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

uint8_t *epd_gram;

static uint8_t power_delay = 0;

static uint8_t init_status = 0;

static TaskHandle_t power_task = NULL;

static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_i80_bus_config_t bus_config = {
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
static esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = -1,
    .pclk_hz = 16000000,
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

static void
epd_power_task(void *pvParameters) {
    for (; init_status;) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if ((power_delay > 0) && (--power_delay) == 0) {
            ESP_ERROR_CHECK(epd_power_off());
        }
    }
    vTaskDelete(NULL);
}

esp_err_t epd_panel_init(void) {
    esp_err_t err;
    if (init_status == 0) {
        err = ESP_OK;
    } else {
        err = ESP_FAIL;
        return err;
    }

    uint32_t gram_size = EPD_H_RES * EPD_V_RES / 4;
    epd_gram = heap_caps_calloc(gram_size, sizeof(uint8_t), MALLOC_CAP_SPIRAM);

    if (epd_gram == NULL) {
        err = ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(epd_board_reset());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(epd_panel_on());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    REG_WRITE(GPIO_ENABLE_W1TS_REG, ((1 << EPD_XCLK_GPIO) |
                                     (1 << EPD_LD_GPIO) |
                                     (1 << EPD_YCLK_GPIO) |
                                     (1 << EPD_YDIO_GPIO) |
                                     (1 << EPD_YOE_GPIO) |
                                     (1 << EPD_XDIO_GPIO)));
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_LD_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_XCLK_GPIO));
    esp_rom_delay_us(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, ((1 << EPD_LD_GPIO) |
                                  (1 << EPD_XCLK_GPIO)));
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

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    err = esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle);
    init_status = 1;
    if (power_task == NULL) {
        xTaskCreate(epd_power_task, "epd power task", 1280, NULL, 1, &power_task);
    }
    return err;
}

esp_err_t epd_send_data(void) {

    if (power_delay == 0) {
        ESP_ERROR_CHECK(epd_power_on());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    power_delay = 10;

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_YDIO_GPIO));
    for (uint16_t row = 0; row < EPD_V_RES; ++row) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_YOE_GPIO));
        esp_rom_delay_us(1);
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << EPD_YCLK_GPIO));
        esp_rom_delay_us(1);
        REG_WRITE(GPIO_OUT_W1TC_REG, ((1 << EPD_YCLK_GPIO) |
                                      (1 << EPD_YOE_GPIO)));
        if (row == 0) {
            REG_WRITE(GPIO_OUT_W1TC_REG, (1 << EPD_YDIO_GPIO));
        };
        esp_lcd_panel_io_tx_param(io_handle, 0x00,
                                  (epd_gram + row * EPD_H_RES / 4),
                                  EPD_H_RES / 4);
    }
    return ESP_OK;
}

esp_err_t epd_panel_del(void) {
    if (init_status == 0) {
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(epd_power_off());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(epd_panel_off());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(io_handle));
    ESP_ERROR_CHECK(esp_lcd_del_i80_bus(i80_bus));
    gpio_reset_pin(EPD_XCLK_GPIO);
    gpio_reset_pin(EPD_LD_GPIO);
    gpio_reset_pin(EPD_YCLK_GPIO);
    gpio_reset_pin(EPD_YDIO_GPIO);
    gpio_reset_pin(EPD_YOE_GPIO);
    gpio_reset_pin(EPD_XDIO_GPIO);
    init_status = 0;
    heap_caps_free(epd_gram);
    epd_gram = NULL;
    return ESP_OK;
}