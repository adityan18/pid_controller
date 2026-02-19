#ifndef PID_MPU6050
#define PID_MPU6050

#include "stm32f4xx_hal.h"

#include <Driver_I2C.h>
#include <stdint.h>
#include <string.h>

/* I2C Configurations */
extern ARM_DRIVER_I2C Driver_I2C1;

/* Standard mode (100 kHz) */
#define I2C_BUS_SPEED ARM_I2C_BUS_SPEED_STANDARD

/* MPU6050 Address Offsets */

#define MPU6050_ADDR 0x68u
#define WHO_AMI_I    0x75u
#define GYRO_CONFIG  0x1Bu
#define ACCEL_CONFIG 0x1Cu
#define PWR_MGMT_1   0x6Bu
#define ACCEL_MEAS   0x3Bu
#define TEMP_MEAS    0x41u
#define GYRO_MEAS    0x43u

/* BITM for accelerometer config and gyroscope config: Bits[4:3] */
#define BITM_A_AND_G_FSEL_CONFIG 0x16u
#define BITP_A_AND_G_FSEL_CONFIG 0x03u

/* Value represents the sensitivity in LSB/g. */
#define AF_SEL_FSEL0_SENSE 16384u /* ±2g */
#define AF_SEL_FSEL1_SENSE 8192u  /* ±4g */
#define AF_SEL_FSEL2_SENSE 4096u  /* ±8g */
#define AF_SEL_FSEL3_SENSE 2048u  /* ±8g */

/* Value represents the sensitivity in LSB/°/s */
#define GF_SEL_FSEL0_SENSE 131  /* ±250°/s */
#define GF_SEL_FSEL1_SENSE 65.5 /* ±500°/s */
#define GF_SEL_FSEL2_SENSE 32.8 /* ±1000°/s */
#define GF_SEL_FSEL3_SENSE 16.4 /* ±2000°/s */

/* osThreadEvent Flags */
#define MPU6050_I2C_TRANSFER_COMPLETE_FLAG (1u << 0u)
#define MPU6050_OTHER_I2C_EVENT_FLAG       (1u << 1u)
/**
 * @brief MPU6050 Sensor Enumeration
 *
 */
typedef enum
{
    MPU6050_ACCEL,
    MPU6050_GYRO,
} mpu6050_sensor_t;

typedef enum
{
    MPU6050_OK         = 0, /* MPU6050 operation successful */
    MPU6050_ERROR      = 1, /* MPU6050 operation failed */
    MPU6050_I2C_ERROR  = 2, /* MPU6050 I2C operation failed */
    MPU6050_RTOS_ERROR = 3, /* MPU6050 RTOS operation failed */
} mpu6050_status_t;

/**
 * @brief DSEL Parameters for Accel/Gyro Full Scale Range Selection
 *
 */
typedef enum
{
    FSEL0,
    FSEL1,
    FSEL2,
    FSEL3,
} mpu6050_fsel_t;

typedef struct __attribute__((packed))
{
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;
} mpu6050_data_t;

mpu6050_status_t pid_mpu6050_Init(void);

#endif /* PID_MPU6050 */
