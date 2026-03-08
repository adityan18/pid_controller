#ifndef PID_COMMON
#define PID_COMMON

#include "pid_mpu6050.h"

#include <cmsis_os2.h>


typedef struct
{
    uint8_t                  index;                 /* Index can be used in future for identifying the data source in case of multiple sensors */
    mpu6050_raw_data_t       sensor_data;           /* Sensor data read from MPU6050 */
    mpu6050_processed_data_t processed_sensor_data; /* Processed sensor data read from MPU6050 */
} pid_sensor_data_queue_t;

#endif /* PID_COMMON */
