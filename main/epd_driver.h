#include "esp_err.h"

#define EPD_H_RES 1024
#define EPD_V_RES 768

extern uint8_t *epd_gram;

esp_err_t epd_panel_init(void);

esp_err_t epd_send_data(void);

esp_err_t epd_panel_del(void);