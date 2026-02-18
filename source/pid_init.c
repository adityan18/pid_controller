#include "stm32f4xx_hal.h"
#include <Driver_USART.h>
#include <cmsis_os2.h>
#include <stdint.h>
#include <stm32f4xx_hal_usart.h>
#include <system_stm32f4xx.h>


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
	array[i] = event;
}

int main()
{

    SystemCoreClockUpdate();

    Driver_USART1.Initialize(uart_cb);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_STOP_BITS_1 | ARM_USART_PARITY_NONE | ARM_USART_FLOW_CONTROL_NONE, 115200);
    Driver_USART1.Control (ARM_USART_CONTROL_TX, 1);
	uint8_t tx_array[10] = {1, 2 , 3, 4, 5 ,6 ,7 ,8 ,9, 0};
	Driver_USART1.Send(&tx_array[0u], sizeof(tx_array));

    while (1)
        ;
    return 0;
}

void init_Gpio(void)
{
    /* Enable RCC to GPIOC */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIO LED, PC13*/
    GPIO_InitTypeDef gpio_c_config = {
        .Pin = GPIO_PIN_13, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW};

    HAL_GPIO_Init(GPIOC, &gpio_c_config);

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (uint16_t i = 0; i < 0xFFFF; i++)
            ;
    }
}