#include "pid_logger.h"
#include "pid_common.h"
#include "pid_mpu6050.h"

#include <Driver_USART.h>

/************************ D A T A ************************/

extern osMessageQueueId_t sensor_to_actuator_processed_queue;
static osEventFlagsId_t uart_logger_thread_event_flags_id;

static osThreadId_t pid_logger_log_processed_data_thread_id;

/************************ P R O T O T Y P E ************************/

static void logger_LogProcessedDataThread(void *arg);
static void logger_UartCallback(uint32_t event);

/************************ P U B L I C  F U N C T I O N S ************************/

bool pid_logger_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    uint32_t usart_control_flags = ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_STOP_BITS_1 |
                                   ARM_USART_PARITY_NONE | ARM_USART_FLOW_CONTROL_NONE;
    if (Driver_USART1.Initialize(logger_UartCallback) != ARM_DRIVER_OK)
    {
        return false;
    }
    else if (Driver_USART1.PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK)
    {
        return false;
    }
    else if (Driver_USART1.Control(usart_control_flags, 921600) != ARM_DRIVER_OK)
    {
        return false;
    }
    else if (Driver_USART1.Control(ARM_USART_CONTROL_TX, 1) != ARM_DRIVER_OK)
    {
        return false;
    }
    else
    {
        pid_logger_log_processed_data_thread_id = osThreadNew(logger_LogProcessedDataThread, NULL, NULL);
    }
    return true;
}

/************************ P R I V A T E  F U N C T I O N S ************************/

static void logger_LogProcessedDataThread(void *arg)
{
    (void)arg;
    while (1)
    {
        pid_sensor_data_queue_t processed_queue_data;
        osStatus_t status =
            osMessageQueueGet(sensor_to_actuator_processed_queue, &processed_queue_data, NULL, osWaitForever);

        if (status == osOK)
        {
            uint8_t buffer[25] = {0u};
            (void)memset(&buffer[0u], 0u, sizeof(buffer));
            /* Transmit the processed sensor data over UART */
            mpu6050_processed_data_t data = processed_queue_data.processed_sensor_data;
            (void)memcpy(&buffer[0u], &data, sizeof(data));
            buffer[24] = '\n'; 
            Driver_USART1.Send(buffer, sizeof(buffer));
            uint32_t os_flags = osEventFlagsWait(uart_logger_thread_event_flags_id,
                                                 LOGGER_UART_TRANSFER_COMPLETE_FLAG | LOGGER_UART_TRANSFER_ERROR_FLAG,
                                                 osFlagsWaitAny, osWaitForever);
        }
    }
}

void logger_UartCallback(uint32_t event)
{
    /* clang-format off */
    switch (event)
    {
        case ARM_USART_EVENT_SEND_COMPLETE:
        {
            osEventFlagsSet(uart_logger_thread_event_flags_id, LOGGER_UART_TRANSFER_COMPLETE_FLAG);
            break;
        }
        default:
		{
            osEventFlagsSet(uart_logger_thread_event_flags_id, LOGGER_UART_TRANSFER_ERROR_FLAG);
			break;
		}
    }
}