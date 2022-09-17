/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "hlw8032.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define reg_total_length 24
#define GET_BIT(x, bit) ((x & (1 << bit)) >> bit) /* 获取第bit位 */
#define GPIO_KEY_PIN_SEL 1ULL << GPIO_POWER_DOWN
// static xSemaphoreHandle  semaphore_handle = NULL;
typedef struct hlw8032
{
    /* data */
    uint8_t State;
    uint8_t check;
    uint32_t voltage_para;
    uint32_t voltage_value;
    uint32_t current_para;
    uint32_t current_value;
    uint32_t power_para;
    uint32_t power_value;
    uint16_t pf;
    uint8_t checksum;
} meter_reg;

typedef struct reg_pf
{
    /* data */
    int pf_overflow_times;
    int updata_7bit_value;
}PF_para;
PF_para PF_parameter;

static QueueHandle_t meter_queue;
static xQueueHandle gpio_isr_queue = NULL;
BaseType_t down_mode_trigger = pdFALSE;

static void meter_data_parse(uint8_t *buffer, uint8_t length);
static uint32_t combine_to3Byte(uint8_t highbyte, uint8_t midbyte, uint8_t lowbyte);

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_isr_queue, &gpio_num, NULL);
}
static void meter_down_task(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_isr_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%d] intr,val:%d\n", io_num, gpio_get_level(io_num));
            if (gpio_get_level(io_num) && !down_mode_trigger)
            {
                // 调用nvs存值
                down_mode_trigger = pdFALSE;
                printf("nvs存储中...\n");
                // write_nvs(electricity);
            }
        }
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(METER_BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(meter_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, METER_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type)
            {
            // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");
                // uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                for (int i = 0; i < event.size; i++)
                {
                    printf("0x%02x ", dtmp[i]);
                }
                printf("\r\n");
                meter_data_parse(dtmp, event.size);
                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(meter_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(meter_queue);
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(EX_UART_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1)
                {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(EX_UART_NUM);
                }
                else
                {
                    uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
/**
 * @brief u32高低字节转换与结合
 *
 * @param highbyte
 * @param midbyte
 * @param lowbyte
 * @return uint32_t
 */
static uint32_t combine_to3Byte(uint8_t highbyte, uint8_t midbyte, uint8_t lowbyte)
{
    return ((uint32_t)((uint32_t)((lowbyte)&0x00FF) +
                       ((uint32_t)((midbyte)&0x00FF) << 8) +
                       ((uint32_t)((highbyte)&0x00FF) << 16)));
}
/**
 * @brief u16字节高低位转换
 *
 * @param highbyte
 * @param lowbyte
 */
static uint16_t combine_to2Byte(uint8_t highbyte, uint8_t lowbyte)
{
    return ((uint16_t)((highbyte)) + ((uint16_t)((lowbyte)&0x00FF) << 8));
}
/**
 * @brief 低位和校验
 *
 * @param buffer
 * @return uint8_t
 */
static uint8_t lowbyte_check(uint8_t *buffer)
{
    uint32_t temp = 0;
    // 除去 State = 0,reg_check=1,checksum=23之后，原数据之和
    for (uint8_t index = 2; index < 23; index++)
    {
        temp += buffer[index];
    }
    uint8_t lowbyte = temp & 0xff;

    printf("low byte = %02x\n", lowbyte);
    return lowbyte;
}
/**
 * @brief 电能数据解析
 *
 * @param buffer
 * @param length
 */
static void meter_data_parse(uint8_t *buffer, uint8_t length)
{
    uint8_t index = 0;
    meter_reg meter_register;
    if (length != reg_total_length)
    {
        printf("数据位错误,length=%d\n", length);
        return;
    }
    meter_register.checksum = buffer[length - 1];
    uint8_t lowbyte = lowbyte_check(buffer);
    if (meter_register.checksum != lowbyte)
    {
        printf("校验未通过,数据丢弃,checksum=%02x lowbyte =%02x\n", meter_register.checksum, lowbyte);
        return;
    }
    switch (buffer[index])
    {
    case 0x55 /* constant-expression */:
        /* code */
        meter_register.check = buffer[1];
        if (meter_register.check != 0x5A)
        {
            printf("检测寄存器错误\n");
            return;
        }
        meter_register.voltage_para = combine_to3Byte(buffer[index + 2], buffer[index + 3], buffer[index + 4]);
        meter_register.voltage_value = combine_to3Byte(buffer[index + 5], buffer[index + 6], buffer[index + 7]);
        double Uc = 1.88; //电压系数
        double Ic = 1;    //电流系数

        meter_register.current_para = combine_to3Byte(buffer[index + 8], buffer[index + 9], buffer[index + 10]);
        meter_register.current_value = combine_to3Byte(buffer[index + 11], buffer[index + 12], buffer[index + 13]);

        meter_register.power_para = combine_to3Byte(buffer[index + 14], buffer[index + 15], buffer[index + 16]);
        meter_register.power_value = combine_to3Byte(buffer[index + 17], buffer[index + 18], buffer[index + 19]);

        bool tmp_updata = GET_BIT(buffer[index + 20], 7);
        if ((!tmp_updata) == PF_parameter.updata_7bit_value)
        {
            PF_parameter.updata_7bit_value = tmp_updata;
            PF_parameter.pf_overflow_times++;
        }
        meter_register.pf = combine_to2Byte(buffer[index + 21], buffer[index + 22]);
        // count_pf 是产生的pf个数，total_pf是需要的个数
        double count_pf = (double)PF_parameter.pf_overflow_times * 65535 + (double)meter_register.pf;
        double total_pf = (1.00 / (double)meter_register.power_para) * (1.00 / (Uc * Ic)) * (10 ^ 9) * 3600;

        electricity.U_rms = (float)meter_register.voltage_para / meter_register.voltage_value * Uc;
        electricity.I_rms = ((float)meter_register.current_para / (float)meter_register.current_value) * Ic;
        electricity.P = ((float)meter_register.power_para / (float)meter_register.power_value) * Uc * Ic;
        electricity.S = electricity.a[0] * electricity.a[1];
        electricity.Q = electricity.a[2] / electricity.a[3];
        if (total_pf == count_pf)
        {
            electricity.a[4]++;
        }
        printf("电压有效值：Ur = %f\n", electricity.U_rms);
        printf("电流有效值：Ir = %f\n", electricity.I_rms);
        printf("有功功率：P = %f\n", electricity.P);
        printf("视在功率S：%f\n", electricity.S);
        printf("功率因素Q：%f\n", electricity.Q);
        printf("当前使用电量：%.2f度\n", electricity.kWh);
        break;
    case 0xaa:
        printf("芯片误差修正功能失效\n");
        return;
    // 电压参数和功率寄存器
    default:
        if (buffer[0] > 0xF0)
        {
            //! 溢出表示电流、电压或功率值非常小，接近0
            printf("State reg 电流、电压或功率值非常小，接近0 :%02x\n", buffer[0]);
            if (GET_BIT(buffer[0], 0))
            {
                printf("电压、电流、功率参数寄存器不可用\n");
                return;
            }
            if (GET_BIT(buffer[0], 1))
            {
                printf("功率寄存器溢出\n");
                return;
            }
            if (GET_BIT(buffer[0], 2))
            {
                printf("电流寄存器溢出\n");
                return;
            }
            if (GET_BIT(buffer[0], 3))
            {
                printf("电压寄存器溢出\n");
                return;
            }
        }
        else
        {
            printf("State reg错误：%02x\n", buffer[0]);
            // TODO 断电后寄存器不会马上清0，处理方式
            return;
        }
        break;
    }
}
/**
 * @brief nvs初始化
 *
 */
void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}
/**
 * @brief nvs写入
 *
 * @param electricity
 */
void write_nvs(Electrical_energy electricity)
{
    printf("\n");
    nvs_handle_t my_handle;
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");

        // write
        printf("Updating elcetricity parameter in NVS ... \n");
        err = nvs_set_u32(my_handle, "U_rms", (uint32_t)(electricity.a[0] * 100));
        err = nvs_set_u32(my_handle, "I_rms", (uint32_t)(electricity.a[1] * 100));
        err = nvs_set_u32(my_handle, "P", (uint32_t)(electricity.a[2] * 100));
        err = nvs_set_u32(my_handle, "S", (uint32_t)(electricity.a[3] * 100));
        err = nvs_set_u32(my_handle, "Q", (uint32_t)(electricity.a[4] * 100));
        err = nvs_set_u32(my_handle, "kWh", (uint32_t)(electricity.a[5] * 100));

        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
        down_mode_trigger = pdTRUE;
    }
}
/**
 * @brief nvs读取
 *
 */
void read_nvs(void)
{
    // Open
    printf("\n");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");
        // Read
        printf("Reading electricity parameter from NVS ... ");
        uint32_t tmp = 0; // value will default to 0, if not set yet in NVS
#ifndef ONLY_READ_kWh_REG
        err = nvs_get_u32(my_handle, "U_rms", &tmp);
        electricity.a[0] = (float)tmp / 100.00;

        err = nvs_get_u32(my_handle, "I_rms", tmp, 10);
        electricity.a[1] = (float)tmp / 100.00;

        err = nvs_get_u32(my_handle, "P", tmp, 10);
        electricity.a[2] = (float)tmp / 100.00;

        err = nvs_get_u32(my_handle, "Q", tmp, 10);
        electricity.a[3] = (float)tmp / 100.00;

        err = nvs_get_u32(my_handle, "S", tmp, 10);
        electricity.a[4] = (float)tmp / 100.00;
#endif
        err = nvs_get_u32(my_handle, "kWh", tmp, 10);
        electricity.a[5] = (float)tmp / 100.00;
        printf("U_rms= %.2f I_rms= %.2f P= %.2f S= %.2f Q= %.2f kWh = %.2f\n",
                electricity.a[0],electricity.a[1],electricity.a[2],electricity.a[3],electricity.a[4],electricity.a[5]);
        switch (err)
        {
        case ESP_OK:
            printf("Done\n");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        nvs_close(my_handle);
    }
}
void app_main(void)
{
    init_nvs();
    read_nvs();
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameter_registers of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 4800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_KEY_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_isr_queue = xQueueCreate(10, sizeof(uint32_t));
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &meter_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, GPIO_UART_TX, GPIO_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    // Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 24);
    // semaphore_handle = xSemaphoreCreateBinary();
    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(meter_down_task, "meter_down_task", 2048, NULL, 12, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_POWER_DOWN, gpio_isr_handler, (void *)GPIO_POWER_DOWN);
    ESP_LOGI(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    while (1)
    {
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
