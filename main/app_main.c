/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP32 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "infra_compat.h"

#include "mqtt_solo.h"
#include "gpioconfig.h"

#include "conn_mgr.h"
#include "driver/gpio.h"
#include "timer_group.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#define GPIO_OUTPUT_IO_0    22
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)  // 配置GPIO_OUT位寄存器
#define GPIO_INPUT_IO_1    23
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_1)  // 配置GPIO_IN位寄存器

static const char* TAG = "app main";

static bool mqtt_started = false;

static esp_err_t wifi_event_handle(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            if (mqtt_started == false) {
                xTaskCreate((void (*)(void *))mqtt_main, "mqtt_example", 10240, NULL, 5, NULL);
                mqtt_started = true;
            }

            break;

        default:
            break;
    }

    return ESP_OK;
}

// static void gpio1_init(void)
// {
//     gpio_config_t io_conf;  // 定义一个gpio_config类型的结构体，下面的都算对其进行的配置

//     io_conf.intr_type = GPIO_PIN_INTR_DISABLE;  // 禁止中断  
//     io_conf.mode = GPIO_MODE_INPUT_OUTPUT;            // 选择输出模式
//     io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // 配置GPIO_OUT寄存器
//     io_conf.pull_down_en = 0;                   // 禁止下拉
//     io_conf.pull_up_en = 0;                     // 禁止上拉
    
//     gpio_config(&io_conf);                      // 最后配置使能
// }

void app_main()
{
    gpio1_init();
    conn_mgr_init();
    conn_mgr_register_wifi_event(wifi_event_handle);
    conn_mgr_set_wifi_config_ext((const uint8_t *)EXAMPLE_WIFI_SSID, strlen(EXAMPLE_WIFI_SSID), (const uint8_t *)EXAMPLE_WIFI_PASS, strlen(EXAMPLE_WIFI_PASS));

    IOT_SetLogLevel(IOT_LOG_INFO);

    conn_mgr_start();
    timer_main();
    
}

