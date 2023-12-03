#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
// #include "ssd1306.h" // https://components.espressif.com/components/espressif/ssd1306

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

static i2c_master_dev_handle_t ags_handle;
// static ssd1306_handle_t ssd_handle;


esp_err_t ags_rread(uint8_t reg, uint32_t *r) {
        uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
        while(true) {
                esp_err_t err;
                if (reg == 0x00) {
                        err = i2c_master_receive(ags_handle, data, 5, 200);
                } else err = i2c_master_transmit_receive(ags_handle, &reg, 1, data, 5, 100);
                // check crc
                bool ok = crc8(data, 4) == data[4];
                bool rdy = data[0] & 0x1;
                if (!ok) {
                        ESP_LOGE("AGS02MA", "CRC error"); 
                        printf("data: %02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4]);
                        printf("crc: %02x\n", crc8(data, 4));
                        vTaskDelay(100);
                        continue;
                }
                if (reg == 0 && !rdy) {
                        ESP_LOGI("AGS02MA", "Not ready, not retrying...");
                        // continue;
                }
                if (err == ESP_ERR_INVALID_ARG) {
                        ESP_LOGE("AGS02MA", "Invalid I2C arg");
                        return err;
                }
                if (err == ESP_ERR_TIMEOUT) {
                        ESP_LOGI("AGS02MA", "I2C timeout");
                        continue;
                }
                ESP_LOGI("AGS02MA", "writing to *r");
                *r = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
                return ESP_OK;
        }
}

// reset zero point
esp_err_t ags_reset() {
        uint8_t data[7] = {0x01, 0x00, 0x0C, 0xFF, 0xF3, 0x00};
        data[6] = crc8(data+2, 4);
        return i2c_master_transmit(ags_handle, data, 7, 100);
}

esp_err_t ags_set_addr(uint8_t addr) {
        uint8_t rdda = 0;
        // rdda is addr bitwise-reversed
        for (int i=0; i<7; i++) {
                rdda |= ((addr>>i) & 0x1) << i;
        }
        uint8_t data[6] = {0x21, addr, rdda, addr, rdda, 0x00};
        data[5] = crc8(data+1, 4);
        return i2c_master_transmit(ags_handle, data, 6, 100);
}


esp_err_t ags_read_version(uint32_t *r) {
        esp_err_t err = ags_rread(0x11, r);
        *r = 0xFF & *r; // keep lowest 8 bits, the rest is reserved might not be zero
        return err;
}

esp_err_t ags_read_hectohm(uint32_t *r) {
        return ags_rread(0x20, r);
}

esp_err_t ags_read(uint32_t *r) { 
        esp_err_t err = ags_rread(0x00, r); 
        uint8_t unit = (*r >> 25) & 0x7;
        if (unit != 0) ESP_LOGE("AGS02MA", "Unit is not PPB CH[2:0] = %d", unit);
        // status was ok, and now unit is PPB logged
        *r <<= 8;
        *r >>= 8;
        return err;
}




void app_main(void)
{
        // initialize i2c at 30 kHz the maximum for AGS02MA
        // also for SSD1306
        i2c_master_bus_config_t conf = {
                .i2c_port = I2C_MASTER_NUM,
                .scl_io_num = 2,
                .sda_io_num = 3,
                .clk_source = I2C_CLK_SRC_DEFAULT, // or I2C_CLK_SRC_RC_FAST
                .glitch_ignore_cnt = 7,
                .flags.enable_internal_pullup = true,
        };
        i2c_device_config_t dev_conf = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = AGS_ADDR,
                .scl_speed_hz = 30000,
        };
        i2c_master_bus_handle_t bus_handle;
        i2c_new_master_bus(&conf, &bus_handle);
        i2c_master_bus_add_device(bus_handle, &dev_conf, &ags_handle);


        // ssd_handle = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
        // ssd1306_refresh_gram(ssd_handle);
        // ssd1306_clear_screen(ssd_handle, 0x00);
        // char data_str[10] = {0};
        // sprintf(data_str, "C STR");
        // ssd1306_draw_string(ssd_handle, 70, 16, (const uint8_t *)data_str, 16, 1);
        // ssd1306_refresh_gram(ssd_handle);


        while(true) {

                uint32_t ppb, hectohm, version;
                ESP_ERROR_CHECK(ags_read(&ppb));
                // ESP_ERROR_CHECK(ags_read_hectohm(&hectohm));
                // ESP_ERROR_CHECK(ags_read_version(&version));

                // printf("PPB: %ld, Hectohm: %ld, Version: %ld\n", (long)ppb, (long)hectohm, (long)version);
                printf("PPB: %ld\n", (long)ppb);
                vTaskDelay(2000 / portTICK_PERIOD_MS);

        }


}
