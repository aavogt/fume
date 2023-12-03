#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "ssd1306.h" // https://components.espressif.com/components/espressif/ssd1306

#define I2C_MASTER_NUM I2C_NUM_0

#define AGS_ADDR 0x1A
#define AGS_WRITE 0x34
#define AGS_READ 0x35


// CRC-8, polynomial x^8+x^5+x^4+1, init 0xff, from the datasheet
// they don't explain whether the CRC also includes the address and register
// bytes, I assume it does not.
uint8_t crc8(uint8_t *data, uint8_t len) {
        uint8_t i,byte,crc = 0xff;
        for (byte=0; byte<len; byte++) {
                crc^=data[byte];
                for(i=0;i<8;i++) {
                        if(crc&0x80) crc=(crc<<1)^0x31;
                        else crc<<=1;
                }
        }
        return crc;
}

static ssd1306_handle_t ssd1306_dev;

void app_main(void)
{
        
        i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = 3,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = 2,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = 30000,
                .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
        };
        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

        ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
        ssd1306_refresh_gram(ssd1306_dev);
        ssd1306_clear_screen(ssd1306_dev, 0x00);

        char data_str[10] = {0};
        while(true) {
                // read PPB from AGS
                uint8_t ags_dat[5];
                i2c_master_read_from_device(I2C_MASTER_NUM,
                                AGS_ADDR,
                                ags_dat, 5, 100);
                // check CRC
                if (crc8(ags_dat, 4) != ags_dat[4]) {
                        ESP_LOGI("AGS02MA", "CRC error\n");
                        continue;
                }
                uint32_t ppb = (ags_dat[1] << 16) | (ags_dat[2] << 8) | ags_dat[3];
                // report ready?

                // display on OLED
                sprintf(data_str, "%ld", ppb);
                ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
                ssd1306_refresh_gram(ssd1306_dev);

                vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
}
