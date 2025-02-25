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

#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "pcnt";

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

volatile int32_t ToTalRotation;//总转速
volatile int MACHINESTATE; // 1启动 0停止
volatile int runtime,fist_runtime,last_runtime;; //RPM每分钟转速,runtime：参考时间 
volatile unsigned long rotationTime = 0;
volatile unsigned long  RPM = 0;
volatile unsigned long firstTime,lastTime;
volatile int StateFlag=0;

/*定时器配置*/
#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    2000
#define LOCAL_DEBUG               1

/*移植pcnt*/
#define PCNT_H_LIM_VAL      5
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    10
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   22  // Pulse Input GPIO

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

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
    unsigned long timeStamp; // The time the event occured
} pcnt_evt_t;
static xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

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
    if(MACHINESTATE==1&&StateFlag>=10&&runtime>=5)
    {
        MACHINESTATE=0;
        StateFlag=0;
        runtime=0;
    }
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}



/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t pcntevt;
    pcntevt.unit = pcnt_unit;
    unsigned long currentMillis = xTaskGetTickCountFromISR();
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    pcntevt.timeStamp = currentMillis; 
    xQueueSendFromISR(pcnt_evt_queue, &pcntevt, &HPTaskAwoken);     // 队列发送
}



static void pcnt_init(int unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,  
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 300);//调大该值可以增加稳定性，但会错过一些信息
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(unit, PCNT_EVT_ZERO);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
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


int getRPm(unsigned long firstTime,unsigned long lastTime)
{
    if(firstTime!=lastTime)
    {
        RPM=lastTime-firstTime;
    }
    else
    {
        StateFlag++;
    }
    return RPM;
}

int getMachineState()
{
    if(0<RPM&&RPM<600) //600为6秒
    {
        MACHINESTATE=1;
    }
    else
    {
        MACHINESTATE=0;
    }
    return MACHINESTATE;
}

void timer_main(void)
{
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    int pcnt_unit = PCNT_UNIT_0;
    tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 1);
    pcnt_init(pcnt_unit);
    int rpm;

    pcnt_evt_t pcntevt;
    portBASE_TYPE res;
    while (1) {
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);
        res = xQueueReceive(pcnt_evt_queue, &pcntevt, 1000 / portTICK_PERIOD_MS);
        if(res == pdTRUE)
        {
            pcnt_get_counter_value(pcnt_unit, &ToTalRotation);
            ESP_LOGI(TAG, "Event PCNT plusetime is :%ld; uint is %d; cnt: %d", pcntevt.timeStamp,pcntevt.unit, ToTalRotation);
            // firstTime=pcntevt.unit
            lastTime=pcntevt.timeStamp;
        }
         else {
            pcnt_get_counter_value(pcnt_unit, &ToTalRotation);
            ESP_LOGI(TAG, "Current counter value :%d", ToTalRotation);
            firstTime=pcntevt.timeStamp;
         }
        rpm=getRPm(firstTime,lastTime);
        getMachineState();
        printf("rpm is %d ;stateflag is %d;runtime is %d;\n",rpm,StateFlag,runtime);
        printf("MACHINESTATE is %d \n",MACHINESTATE);

    }
}
