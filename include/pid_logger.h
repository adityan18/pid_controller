#ifndef PID_LOGGER
#define PID_LOGGER

#include "pid_common.h"

#include <Driver_USART.h>

/* I2C Configurations */
extern ARM_DRIVER_USART Driver_USART1;

#define LOGGER_UART_TRANSFER_COMPLETE_FLAG (1u << 0u)
#define LOGGER_UART_TRANSFER_ERROR_FLAG    (1u << 1u)

bool pid_logger_Init(void);

#endif /* PID_LOGGER */
