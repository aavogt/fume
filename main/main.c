#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "ssd1306.h" // https://components.espressif.com/components/espressif/ssd1306
#include "pid_ctrl.h" // https://components.espressif.com/components/espressif/pid_ctrl

#define I2C_MASTER_NUM I2C_NUM_0

#define AGS_ADDR 0x1A
#define AGS_WRITE 0x34
#define AGS_READ 0x35

#define LEDC_RESOLUTION 10


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

static uint32_t counter_diff = 0;

void counter_isr(void *arg) {
        // tick counts for the last 10 pulses -- nothing stored at ts[0]
        static uint32_t ts[11] = {0};
        static uint8_t old = 0, cur; // index into the current and the oldest tick count
        cur = old++;
        old = 1 + ((old-1) % 11);
        ts[cur] = xTaskGetTickCountFromISR();
        counter_diff = ts[cur] - ts[old];
}

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


        // GPIO6 reads the fan tachometer
        // the isr stores the number of ticks for the last 10 pulses
        // in uint32_t counter_diff
        gpio_config_t io_conf = {
                .intr_type = GPIO_INTR_POSEDGE,
                .mode = GPIO_MODE_INPUT,
                .pin_bit_mask = (1ULL<<6),
                .pull_down_en = 1,
                .pull_up_en = 0,
        };
        gpio_config(&io_conf);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(6, counter_isr, NULL);

        // GPIO5 is PWM by LEDC to adjust the XL4015 buck converter output voltage
        ledc_timer_config_t ledc_timer = {
            .freq_hz = 1000,
            .duty_resolution = LEDC_RESOLUTION,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = GPIO_NUM_5,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        };
        ledc_channel_config(&ledc_channel);

        // pid controller
        // suppose the fan has a time constant of 0.1s
        // and there is a time delay of 0.01 s. Then via a Pade approximation
        // the ZN gain is Kc*Kp = (2/a) + (1/tau) = 210
        // (see wxmaxima)
        // but what is the process gain Kp? It is the max value of
        // counter_diff divided by the max PWM duty cycle (1024).
        // If I can get 3000 RPM and one pulse per revolution, and
        // one tick per 10 ms, then Kp = 3000/60/1024/0.01 = 4.88
        // but line doesn't go through the origin: that is the fan
        // will get stuck at a duty cycle somewhere above 0. This may
        // lead to Kp=3 since the 0 to 3000 RPM is covered by a smaller
        // PWM range (1024 - 300 say). But I want a less aggressive
        // controller, so I will stick with a higher Kp:
        //
        // if Kp = 5, Kcu = 210/5 = 42
        //
        // ZN also needs the oscillation period.
        // The pole has imaginary part = 210.
        // Then the period Tu = 2pi/210 = 0.03
        //
        // Substitute into PID 0.6Ku, 1.2Ku/Tu, 3KuTu/40:
        pid_ctrl_config_t pid_config = {
          .init_param = {
                .kp = 25,
                .ki = 1680,
                .kd = 9.45e-2,
                .max_output = 1023,
                .min_output = 0,
                .max_integral = 0.1,
                .min_integral = -0.1,
                .cal_type = PID_CAL_TYPE_POSITIONAL,
        }};
        pid_ctrl_block_handle_t pid_handle;
        pid_new_control_block(&pid_config, &pid_handle);


        float fduty = 0, ferr = 0;

        // put this in the ISR? In a separate task? How often to take action?
        uint32_t counter_setpoint = 10000;
        ferr = (float)counter_setpoint - (float)counter_diff;
        pid_compute(pid_handle, ferr, &fduty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)fduty);

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
