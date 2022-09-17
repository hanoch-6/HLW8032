/**
 * @file hlw8032.h
 * @author HuangMiaozhi (hanoch1024@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-12
 * 
 * @attention
 * Jinxin Microelec co.
 * 
 */
/*GPIO启用定义*/
#define EX_UART_NUM UART_NUM_1
#define GPIO_UART_TX 19
#define GPIO_UART_RX 16
#define GPIO_POWER_DOWN 18
#define BUF_SIZE (512)
#define METER_BUF_SIZE (BUF_SIZE)

#define ONLY_READ_kWh_REG 1

typedef struct Electrical_energy
{
    float a[0];
    float U_rms;//电压有效值
    float I_rms;//电流有效值
    float P;//有功功率
    float S;//视在功率
    float Q;//功率因素
    float kW_h;//电量，度
}Electrical_energy;

Electrical_energy electricity;

void write_nvs(Electrical_energy electricity);
void read_nvs(void);