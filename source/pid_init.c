#include "pid_common.h"
#include "pid_logger.h"
#include "pid_mpu6050.h"
#include "pid_pwm.h"

#include <Driver_USART.h>
#include <stm32f4xx_hal.h.>
#include <system_stm32f4xx.h>

#include <stdint.h>
#include <string.h>

/************************ D A T A ************************/

osMessageQueueId_t sensor_to_actuator_raw_queue;
osMessageQueueId_t sensor_to_actuator_processed_queue;

/************************ P R O T O T Y P E ************************/

/************************ P U B L I C  F U N C T I O N S ************************/

/************************ P R I V A T E  F U N C T I O N S ************************/

extern ARM_DRIVER_USART Driver_USART1;

/**
 * @brief Initializes the RCC (Reset and Clock Control) to set up the system clock.
 *
 */
void init_Rcc(void);

/**
 * @brief Initializes the GPIO (General Purpose Input/Output) for the specified pin and configuration.
 *
 */
void init_Gpio(void);

uint32_t array[10] = {0u};
void uart_cb(uint32_t event)
{
    static uint8_t i = 0;
    array[i++] = event;
}

uint32_t i2c_array[10] = {0u};
void i2c_cb(uint32_t event)
{
    static uint8_t j = 0;
    i2c_array[j++] = event;
}

void uart_tx()
{
    Driver_USART1.Initialize(uart_cb);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_STOP_BITS_1 |
                              ARM_USART_PARITY_NONE | ARM_USART_FLOW_CONTROL_NONE,
                          115200);
    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
    uint8_t tx_array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
    Driver_USART1.Send(&tx_array[0u], sizeof(tx_array));
}

void i2c_read()
{
}

void pwm_generate()
{
    /* Enable RCC to GPIOC */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure GPIO LED, PC13*/
    GPIO_InitTypeDef gpio_a_config = {.Pin = GPIO_PIN_6,
                                      .Mode = GPIO_MODE_AF_PP,
                                      .Pull = GPIO_NOPULL,
                                      .Speed = GPIO_SPEED_FREQ_LOW,
                                      .Alternate = GPIO_AF2_TIM3};

    HAL_GPIO_Init(GPIOA, &gpio_a_config);

    uint32_t CCR1_res = 0x1F4;
    TIM3->PSC = 16 - 1;
    TIM3->ARR = 20000 - 1; /* T = 20ms */
    TIM3->CCR1 = CCR1_res; /* DC = 3ms */

    TIM3->CCMR1 |= (6 << 4); // PWM mode 1
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;
    /* 0x280, 0x5D0, 0x960 */

    uint16_t cal_array[] = {0x280, 0x5D0, 0x960};
    while (1)
    {
        static uint8_t i = 0;
        TIM3->CCR1 = cal_array[i++];
        if (i == 3u)
            i = 0u;
        for (uint32_t j = 0; j < 0xFFFFF; j++)
            ;
    }
}

void init_Clock(void)
{
    // 1. Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // 2. Configure PLL
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |  // PLLM = 25
                   (336 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 336
                   (1 << RCC_PLLCFGR_PLLP_Pos) |   // PLLP = 4 (00=2, 01=4)
                   RCC_PLLCFGR_PLLSRC_HSE;         // HSE as source

    // 3. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // 4. Configure Flash latency (VERY IMPORTANT)
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS; // 84MHz requires 2 wait states

    // 5. Set prescalers
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB = 84 MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 = 42 MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = 84 MHz

    // 6. Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
}

int main()
{

    init_Clock();
    SystemCoreClockUpdate();

    osKernelInitialize();
    osMessageQueueAttr_t sensor_to_actuator_raw_queue_attr = {
        .name = "sensor_to_actuator_raw_queue",
    };
    osMessageQueueAttr_t sensor_to_actuator_processed_queue_attr = {
        .name = "sensor_to_actuator_processed_queue",
    };
    sensor_to_actuator_raw_queue =
        osMessageQueueNew(10, sizeof(pid_sensor_data_queue_t), &sensor_to_actuator_raw_queue_attr);
    sensor_to_actuator_processed_queue =
        osMessageQueueNew(10, sizeof(pid_sensor_data_queue_t), &sensor_to_actuator_processed_queue_attr);

    init_Gpio();
    pid_mpu6050_Init();
    pid_logger_Init();
    pid_pwm_Init();
    //    pwm_generate();

    osKernelStart();

    return 0;
    ;
}

/* RTOS Idle Thread */
void osRtxidleThread(void *arg)
{
    (void)arg;
    for (;;)
    {
        __NOP();
    }
}

void init_Gpio(void)
{
    /* Enable RCC to GPIOC */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIO LED, PC13*/
    GPIO_InitTypeDef gpio_c_config = {
        .Pin = GPIO_PIN_13, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW};

    HAL_GPIO_Init(GPIOC, &gpio_c_config);
    HAL_GPIO_WritePin(GPIOC, 16, GPIO_PIN_RESET);
    //    while (1)
    //    {
    //        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    //        for (uint16_t i = 0; i < 0xFFFF; i++)
    //            ;
    //    }
}