/* General Purpose Timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "timer_group.h"
#include "gpioconfig.h"

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define GPIO_INPUT_IO_0    22
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)  // 配置GPIO_IN位寄存器
// #define ESP_INTR_FLAG_DEFAULT 0 //中断标志位

volatile int debounceCounter; //总转速
volatile bool debouncetflag;  //消抖标志位
volatile bool MACHINESTATE; // 1启动 0停止
volatile int rpm,runtime,fist_runtime,last_runtime;; //rpm每分钟转速,runtime：参考时间 

#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    2000
#define LOCAL_DEBUG               1

volatile unsigned long rotationTime = 0;

// uint32_t RPM       = 0;
// uint32_t avgRPM    = 0;
// float RPM       = 0;
// float avgRPM    = 0;

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

static xQueueHandle s_timer_queue;

/**
 * @brief GPIO初始化接近开关使用D22引脚
 *
 */
void gpio1_init(void)
{
    gpio_config_t io_conf;  // 定义一个gpio_config类型的结构体，下面的都算对其进行的配置

    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;  // GPIO23低电平触发中断 
    io_conf.mode = GPIO_MODE_INPUT;            // 选择输入模式
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // 配置GPIO_OUT寄存器
    io_conf.pull_down_en = 0;                   // 禁止下拉
    io_conf.pull_up_en = 0;                     // 禁止上拉
    gpio_config(&io_conf);                      // 最后配置使能

}
/**
 * @brief 定时器中断回调函数
 *  
 *
 */
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
        example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };
    /*每分钟转速待优化*/
    runtime++;
    if(runtime==1)
    {
        fist_runtime=0;
        fist_runtime= debounceCounter;  
    }
    if(runtime==11)
    {
        last_runtime = debounceCounter;
        runtime=0;
        rpm = 6*(last_runtime - fist_runtime);
        last_runtime = 0;
        
    }

    if(gpio_get_level(GPIO_NUM_22)==0&&debouncetflag==1) // 只有满足gpio检测到低电平和标志位为1时才开始计数。
    {
        debounceCounter++; //转速值
        rotationTime=0;  
        debouncetflag=0;   //旋转时计数标志位为0
        MACHINESTATE=1;   //旋转时机器状态为1，即机器正常工作
    }
    if(gpio_get_level(GPIO_NUM_22)==1)
    {
        rotationTime++;
        if(rotationTime>=2)
        {
            debouncetflag=1; //计数标志位，两次脉冲间隔大于2S，标志位值1
        }
        else
        {
            debouncetflag=0; //计数标志位，两次脉冲间隔小于等于2秒，即非圆机转速正常状态，标志位为0
        }
        if(rotationTime>20) //rotationTime如果20秒未清零则判断其已停机。
        {
            MACHINESTATE=0; //停止是机器状态为0，即机器停转
        }
    }
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}

void timer_main(void)
{
    void gpio1_init(void);
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));

    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 1);
    // example_tg_timer_init(TIMER_GROUP_1, TIMER_0, false, 5);

    while (1) {
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);
        // printf("totoration is %d \n",debounceCounter);
        // // // printf("rotationTime is %ld \n",rotationTime);

        // // //  printf("debouncetime is %d \n",debouncetime);
        // printf("MACHINESTATE is %d \n",MACHINESTATE);
        // printf("rpm is %d \n",rpm);

    }
}
