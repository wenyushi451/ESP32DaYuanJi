#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOSConfig.h"
#include "driver/gpio.h"
#include "gpioconfig.h"

#define GPIO_INPUT_IO_0    22
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)  // 配置GPIO_OUT位寄存器
#define ESP_INTR_FLAG_DEFAULT 0 //中断标志位

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

}

//static void gpio_task()
 static void   gpio_task(void* arg)
{
    uint32_t io_num;
    uint32_t i = 0;
    for(;;) {
         if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // if(gpio_get_level(GPIO_NUM_22)==0){
                 i++;
            //     // printf("total_rotation is : %d\n",i);
                 vTaskDelay(1000 / portTICK_RATE_MS);
             }
            }
}

void gpio1_init(void)
{
    gpio_config_t io_conf;  // 定义一个gpio_config类型的结构体，下面的都算对其进行的配置

    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // GPIO23低电平触发中断 
    io_conf.mode = GPIO_MODE_INPUT;            // 选择输入模式
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // 配置GPIO_OUT寄存器
    io_conf.pull_down_en = 0;                   // 禁止下拉
    io_conf.pull_up_en = 0;                     // 禁止上拉
    gpio_config(&io_conf);                      // 最后配置使能

}



void gpio_main()
{
    gpio_config_t io_conf = {};
    gpio1_init();
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);//change gpio intrrupt type for one pin
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));//create a queue to handle gpio event from isr
    xTaskCreate(gpio_task, "gpio_task_example", 2048, NULL, 10, NULL);//start gpio task
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);//install gpio isr service
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);


    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0); 
    int cnt = 0;
    while(1){
        
        // printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
