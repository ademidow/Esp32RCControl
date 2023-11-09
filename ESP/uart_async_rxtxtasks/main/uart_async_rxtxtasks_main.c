/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "modbus.h"

#define TXD_PIN (GPIO_NUM_23)
#define RXD_PIN (GPIO_NUM_22)

#define WHEEL_PIN (GPIO_NUM_19)
#define SPEED_PIN (GPIO_NUM_18)
#define BUTN1_PIN (GPIO_NUM_17)
#define BUTN2_PIN (GPIO_NUM_16)

#define GPIO_INPUT_PIN_SEL  ((1ULL<<WHEEL_PIN) | (1ULL<<SPEED_PIN) | (1ULL<<BUTN1_PIN) | (1ULL<<BUTN2_PIN))

#define ESP_INTR_FLAG_DEFAULT 0

#define TIMER_DIVIDER (16)
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)

static const int RX_BUF_SIZE = 1024;

unsigned short wheel;

extern short mb_data[];
extern unsigned short state;

static xQueueHandle gpio_evt_queue = NULL;

struct  xMessage
{
    unsigned short pinNum;
    unsigned long timeStamp;
    unsigned char value;   
} xmsg;

struct TStamp
{
    unsigned long raiseTime;
    unsigned long failTime;
    unsigned int pulseTime;
    unsigned int maxPulse;
    unsigned int minPulse;
    unsigned int zerroPulse;
    unsigned int deadTime;
    int maxValue;
    int minValue;
    int value;
} tstmp;

struct TStamp rcControl[4];  // 0 - WHEEl       1 - SPEED       2 - BUTTON1        3 - BUTTON2

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    struct xMessage msg;
    msg.pinNum = (unsigned short)((uint32_t) arg);
    msg.value = gpio_get_level(msg.pinNum);
    msg.timeStamp = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    xQueueSendFromISR(gpio_evt_queue, &msg, NULL);
}

void init(void) 
{

    rcControl[0].deadTime = 100;
    rcControl[0].maxValue = 100;
    rcControl[0].minValue = -100;
    rcControl[0].maxPulse = 10000;
    rcControl[0].minPulse = 4800;
    rcControl[0].zerroPulse = 7425;
    rcControl[0].pulseTime = 7425;

    
    rcControl[1].deadTime = 100;
    rcControl[1].maxValue = 100;
    rcControl[1].minValue = -100;
    rcControl[1].maxPulse = 10000;
    rcControl[1].minPulse = 4800;
    rcControl[1].zerroPulse = 7425;
    rcControl[1].pulseTime = 7425;

    
    rcControl[2].deadTime = 75;
    rcControl[2].maxValue = 100;
    rcControl[2].minValue = -100;
    rcControl[2].maxPulse = 10000;
    rcControl[2].minPulse = 4800;
    rcControl[2].zerroPulse = 7425;
    rcControl[2].pulseTime = 7425;

    
    rcControl[3].deadTime = 75;
    rcControl[3].maxValue = 100;
    rcControl[3].minValue = -100;
    rcControl[3].maxPulse = 10000;
    rcControl[3].minPulse = 4800;
    rcControl[3].zerroPulse = 7425;
    rcControl[3].pulseTime = 7425;

    timer_config_t config = {
      .divider = TIMER_DIVIDER,
      .counter_dir = TIMER_COUNT_UP,
      .counter_en = TIMER_PAUSE,
      .alarm_en = TIMER_ALARM_EN,
      .auto_reload = true,
  };
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);

    const uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void sendMBReq()
{
    MbSendGetHRSRQ(1, 0, 16, UART_NUM_1, 0);
    while (state == MODBUS_STATES_RQSEND)
        vTaskDelay(10 / portTICK_PERIOD_MS);
    MbSendGetHRSRQ(2, 0, 16, UART_NUM_1, 16);
    while (state == MODBUS_STATES_RQSEND)
        vTaskDelay(10 / portTICK_PERIOD_MS);
    MbSendGetHRSRQ(3, 0, 16, UART_NUM_1, 32);
    while (state == MODBUS_STATES_RQSEND)
        vTaskDelay(10 / portTICK_PERIOD_MS);
    short val = 0;
    if (rcControl[2].value>0)
        val = val | 8;
    if (rcControl[3].value>0)
        val = val | 16;

    MbSendSetRegister(3, 0, val, UART_NUM_1);
    while (state == MODBUS_STATES_RQSEND)
        vTaskDelay(10 / portTICK_PERIOD_MS);

    val = rcControl[1].value*2 + rcControl[0].value/1;
    MbSendSetRegister(0, 5, val, UART_NUM_1);
    while (state == MODBUS_STATES_RQSEND)
        vTaskDelay(10 / portTICK_PERIOD_MS);

    val = rcControl[1].value*2 - rcControl[0].value/1;
    MbSendSetRegister(0, 6, val, UART_NUM_1);
    while (state == MODBUS_STATES_RQSEND)
        vTaskDelay(10 / portTICK_PERIOD_MS);

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

static void tx_task(void *arg)
{
    while (1) {
        sendMBReq();
        vTaskDelay(5 / portTICK_PERIOD_MS);        
    }
}

static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        if (state == MODBUS_STATES_RQSEND)
        {
            const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, MODBUS_WAIT_TIME / portTICK_RATE_MS);
            MbParseAnswer(data, rxBytes);
        }
        else 
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    free(data);
}

static void gpio_task(void* arg)
{

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(16, sizeof(xmsg));

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //set intrrupt type for pins
    //hook isr handler for specific gpio pin
    gpio_set_intr_type(WHEEL_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(WHEEL_PIN, gpio_isr_handler, (void*) WHEEL_PIN);

    gpio_set_intr_type(SPEED_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(SPEED_PIN, gpio_isr_handler, (void*) SPEED_PIN);
    
    gpio_set_intr_type(BUTN1_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(BUTN1_PIN, gpio_isr_handler, (void*) BUTN1_PIN);
    
    gpio_set_intr_type(BUTN2_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(BUTN2_PIN, gpio_isr_handler, (void*) BUTN2_PIN);

    struct xMessage io_msg;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_msg, portMAX_DELAY)) 
        {
            int pos = -1;
            if (io_msg.pinNum == WHEEL_PIN) pos = 0;
            if (io_msg.pinNum == SPEED_PIN) pos = 1;
            if (io_msg.pinNum == BUTN1_PIN) pos = 2;
            if (io_msg.pinNum == BUTN2_PIN) pos = 3;
            if (pos >=0)
            {
                if (io_msg.value == 1)
                {
                    rcControl[pos].raiseTime = io_msg.timeStamp;
                }
                if (io_msg.value == 0)
                {
                    rcControl[pos].failTime = io_msg.timeStamp;
                    rcControl[pos].pulseTime = (unsigned int)(rcControl[pos].failTime - rcControl[pos].raiseTime);
                    if (rcControl[pos].pulseTime > (rcControl[pos].maxPulse  + 2*rcControl[pos].deadTime) || rcControl[pos].pulseTime < (rcControl[pos].minPulse - 2*rcControl[pos].deadTime)) 
                        rcControl[pos].pulseTime = rcControl[pos].zerroPulse;
                    if (rcControl[pos].pulseTime > rcControl[pos].zerroPulse) 
                    {
                        if (rcControl[pos].pulseTime - rcControl[pos].zerroPulse < rcControl[pos].deadTime)
                        {
                            rcControl[pos].pulseTime = rcControl[pos].zerroPulse;
                            rcControl[pos].value = 0;
                        }
                        else
                        {
                            if (rcControl[pos].pulseTime > rcControl[pos].maxPulse - rcControl[pos].deadTime)
                                rcControl[pos].value = rcControl[pos].maxValue;
                            else
                                rcControl[pos].value = (int)(rcControl[pos].maxValue*(rcControl[pos].pulseTime - rcControl[pos].zerroPulse)/(rcControl[pos].maxPulse - rcControl[pos].zerroPulse));
                        }
                    }
                    if (rcControl[pos].pulseTime < rcControl[pos].zerroPulse) 
                    {
                        if (rcControl[pos].zerroPulse - rcControl[pos].pulseTime < rcControl[pos].deadTime)
                        {
                            rcControl[pos].pulseTime = rcControl[pos].zerroPulse;
                            rcControl[pos].value = 0;
                        }
                        else 
                        {
                            if (rcControl[pos].pulseTime < rcControl[pos].minPulse + rcControl[pos].deadTime)
                                rcControl[pos].value = rcControl[pos].minValue;
                            else 
                                rcControl[pos].value = (int)(rcControl[pos].minValue*((int)rcControl[pos].zerroPulse - (int)rcControl[pos].pulseTime)/(int)(rcControl[pos].zerroPulse - rcControl[pos].minPulse));
                        }
                    }
                    
                }
            }
        }
    }
}

static void debug_tasg(void* arg)
{
    static const char *DB_TASK_TAG = "DB_TASK";
    esp_log_level_set(DB_TASK_TAG, ESP_LOG_INFO);
        while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //unsigned int val0 = 0;
        int val1 = 0;
        int val2 = 0;
        int val3 = 0;
        int val4 = 0;

        val1 = mb_data[MODBUS_STATUS_R];
        val2 = mb_data[MODBUS_STATUS_R+1];
        val3 = mb_data[MODBUS_STATUS_R+2];
        ESP_LOGI(DB_TASK_TAG, "\n Modbus states: 1 - %d,  2 - %d,  3 - %d", val1, val2, val3);
        
        val1 = mb_data[3];
        val2 = mb_data[16+3];
        val3 = mb_data[4];
        val4 = mb_data[16+4];
        ESP_LOGI(DB_TASK_TAG, "\n SPEED: RIGHT_1: %d,  LEFT_2: %d,  LEFT_1: %d,  RIGHT_2: %d", val1, val2, val3, val4);
        
        val1 = mb_data[5];
        val2 = mb_data[16+5];
        val3 = mb_data[6];
        val4 = mb_data[16+6];
        ESP_LOGI(DB_TASK_TAG, "\n SET_SPEED: RIGHT1: %d,  LEFT_2: %d,  LEFT_1: %d,  RIGHT_2: %d", val1, val2, val3, val4);

        val1 = rcControl[0].pulseTime;
        val2 = rcControl[0].value;
        ESP_LOGI(DB_TASK_TAG, "WHELL pulse: %d,  value: %d", val1,  val2);
        val1 = rcControl[1].pulseTime;
        val2 = rcControl[1].value;
        ESP_LOGI(DB_TASK_TAG, "SPEED pulse: %d,  value: %d", val1,  val2);
        val1 = rcControl[2].pulseTime;
        val2 = rcControl[2].value;
        ESP_LOGI(DB_TASK_TAG, "BUTTON1 pulse: %d,  value: %d", val1,  val2);
        val1 = rcControl[3].pulseTime;
        val2 = rcControl[3].value;
        ESP_LOGI(DB_TASK_TAG, "BUTTON2 pulse: %d,  value: %d", val1,  val2);
        
        
    }
}

void app_main(void)
{
    wheel = 0;
    init();
    
    xTaskCreate(rx_task, "uart_rx_task", 2048, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(debug_tasg, "debug_task", 2048, NULL, configMAX_PRIORITIES-3, NULL);
}
