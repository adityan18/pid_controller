#include "pid_mpu6050.h"

#include <cmsis_os2.h>
#include <string.h>

/************************ D A T A ************************/

mpu6050_fsel_t af_sel = FSEL0; /* Default Accelerometer Full Scale Selection */
mpu6050_fsel_t gf_sel = FSEL0; /* Default Gyroscope Full Scale Selection */

float accel_sensitivity[] = {
    AF_SEL_FSEL0_SENSE,
    AF_SEL_FSEL1_SENSE,
    AF_SEL_FSEL2_SENSE,
    AF_SEL_FSEL3_SENSE,
};

float gyro_sensitivity[] = {
    GF_SEL_FSEL0_SENSE,
    GF_SEL_FSEL1_SENSE,
    GF_SEL_FSEL2_SENSE,
    GF_SEL_FSEL3_SENSE,
};

static osThreadId_t mpu6050_sample_sensor_data_thread_id;
/************************ P R O T O T Y P E ************************/

static mpu6050_status_t mpu6050_SetupI2C(void);
static mpu6050_status_t mpu6050_WhoAmICheck(void);
static void mpu6060_I2CCallback(uint32_t event);
static mpu6050_status_t mpu6050_ConfigureSensor(mpu6050_sensor_t sensor, mpu6050_fsel_t fsel);
static mpu6050_status_t mpu6050_ConfigurePwr(uint8_t pwr_cfg);
static void mpu6050_ReadSensorDataTask(void *arg);
static bool mpu6060_RtosInit(void);

/************************ P U B L I C  F U N C T I O N S ************************/
mpu6050_status_t pid_mpu6050_Init(void)
{
    mpu6050_status_t status = MPU6050_ERROR;

    if (mpu6050_SetupI2C() != MPU6050_OK)
    {
        status = MPU6050_I2C_ERROR;
    }
    else if (mpu6050_WhoAmICheck() != MPU6050_OK)
    {
        status = MPU6050_ERROR;
    }
    else if (mpu6050_ConfigureSensor(MPU6050_ACCEL, af_sel) != MPU6050_OK)
    {
        status = MPU6050_ERROR;
    }
    else if (mpu6050_ConfigureSensor(MPU6050_GYRO, gf_sel) != MPU6050_OK)
    {
        status = MPU6050_ERROR;
    }
    else if (mpu6050_ConfigurePwr(0x00) != MPU6050_OK)
    {
        status = MPU6050_ERROR;
    }
    else if (mpu6060_RtosInit() != true)
    {
        status = MPU6050_RTOS_ERROR;
    }
    else
    {
        status = MPU6050_OK;
    }

    return status;
}

/************************ P R I V A T E  F U N C T I O N S ************************/

static void mpu6060_I2CCallback(uint32_t event)
{
    /* clang-format off */
    switch (event)
    {
        case ARM_I2C_EVENT_TRANSFER_DONE: {
            osThreadFlagsSet(mpu6050_sample_sensor_data_thread_id, MPU6050_I2C_TRANSFER_COMPLETE_FLAG);
            break;
        }
        case ARM_I2C_EVENT_TRANSFER_INCOMPLETE: /* If received abort and retry */
        case ARM_I2C_EVENT_SLAVE_RECEIVE: /* Always operates in master mode, not expected */
        case ARM_I2C_EVENT_SLAVE_TRANSMIT: /* Always operates in master mode, not expected */
        case ARM_I2C_EVENT_GENERAL_CALL: /* Always operates in master mode, not expected */
        default: {
            osThreadFlagsSet(mpu6050_sample_sensor_data_thread_id, MPU6050_OTHER_I2C_EVENT_FLAG);
            break;
        }
    }
    /* clang-format on */
}

static mpu6050_status_t mpu6050_SetupI2C(void)
{
    mpu6050_status_t status = MPU6050_I2C_ERROR;

    /* Enable RCC Clock for GPIOB */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    if (Driver_I2C1.Initialize(mpu6060_I2CCallback) != ARM_DRIVER_OK)
    {
        status = MPU6050_I2C_ERROR;
    }
    else if (Driver_I2C1.PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK)
    {
        status = MPU6050_I2C_ERROR;
    }
    else if (Driver_I2C1.Control(ARM_I2C_BUS_SPEED, I2C_BUS_SPEED) != ARM_DRIVER_OK)
    {
        status = MPU6050_I2C_ERROR;
    }
    else
    {
        status = MPU6050_OK;
    }

    return status;
}

static mpu6050_status_t mpu6050_WhoAmICheck(void)
{
    mpu6050_status_t result = MPU6050_ERROR;

    /* Read WHO_AM_I Register */
    uint8_t tx_array[1u] = {WHO_AMI_I};
    Driver_I2C1.MasterTransmit(MPU6050_ADDR, &tx_array[0u], sizeof(tx_array), true);
    while (Driver_I2C1.GetStatus().busy)
    {
    };

    uint8_t rx_array[8u] = {0u};
    Driver_I2C1.MasterReceive(MPU6050_ADDR, &rx_array[0u], sizeof(rx_array), false);
    while (Driver_I2C1.GetStatus().busy)
    {
    };

    /* Check if Slave Address is Matching */
    if (rx_array[0u] == MPU6050_ADDR)
    {
        result = MPU6050_OK;
    }
    return result;
}

static mpu6050_status_t mpu6050_ConfigureSensor(mpu6050_sensor_t sensor, mpu6050_fsel_t fsel)
{
    uint8_t config_reg = 0u;

    /* clang-format off */
    switch (sensor)
    {
        case MPU6050_ACCEL:
        {
            config_reg = ACCEL_CONFIG;
            break;
        }
        case MPU6050_GYRO:
        {
            config_reg = GYRO_CONFIG;
            break;
        }
        default:
        {
            break;
        }
    }
    /* clang-format on */

    uint8_t sensor_config = (uint8_t)fsel << BITP_A_AND_G_FSEL_CONFIG;
    uint8_t tx_array[2u] = {config_reg, sensor_config};
    Driver_I2C1.MasterTransmit(MPU6050_ADDR, &tx_array[0u], sizeof(tx_array), false);
    while (Driver_I2C1.GetStatus().busy)
    {
    };
    return MPU6050_OK;
}

static mpu6050_status_t mpu6050_ConfigurePwr(uint8_t pwr_cfg)
{
    uint8_t tx_array[2u] = {PWR_MGMT_1, pwr_cfg};
    Driver_I2C1.MasterTransmit(MPU6050_ADDR, &tx_array[0u], sizeof(tx_array), false);
    while (Driver_I2C1.GetStatus().busy)
    {
    };
    return MPU6050_OK;
}

static bool mpu6060_RtosInit(void)
{
    bool thread_creation_successful = false;
    mpu6050_sample_sensor_data_thread_id = osThreadNew(mpu6050_ReadSensorDataTask, NULL, NULL);
    if (mpu6050_sample_sensor_data_thread_id != NULL)
    {
        thread_creation_successful = true;
    }
    return thread_creation_successful;
}

static void mpu6050_ReadSensorDataTask(void *arg)
{
    bool sensor_read_successful = false;
    while (1)
    {
        static uint8_t tx_array[2u] = {ACCEL_MEAS, GYRO_MEAS};
        /* AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L */
        static uint16_t rx_array[6u] = {0u};
        mpu6050_data_t sensor_data = {0u};

        for (uint8_t i = 0u; i < 2u; i++)
        {
            sensor_read_successful = true;
            uint32_t os_flags;

            Driver_I2C1.MasterTransmit(MPU6050_ADDR, &tx_array[i], 1u, true);
            os_flags = osThreadFlagsWait(MPU6050_I2C_TRANSFER_COMPLETE_FLAG | MPU6050_OTHER_I2C_EVENT_FLAG,
                                         osFlagsWaitAny, osWaitForever);
            if ((os_flags & MPU6050_OTHER_I2C_EVENT_FLAG) != 0u)
            {
                sensor_read_successful = false;
                break;
            }

            Driver_I2C1.MasterReceive(MPU6050_ADDR, (uint8_t *)&rx_array[3u * i], 6u, false);
            os_flags = osThreadFlagsWait(MPU6050_I2C_TRANSFER_COMPLETE_FLAG | MPU6050_OTHER_I2C_EVENT_FLAG,
                                         osFlagsWaitAny, osWaitForever);
            if ((os_flags & MPU6050_OTHER_I2C_EVENT_FLAG) != 0u)
            {
                sensor_read_successful = false;
                break;
            }
        }

        if (sensor_read_successful == true)
        {
            /* Reverse the 16-bit data as endianess is reversed because of how the data is transferred from MPU6050 */
            for (uint8_t i = 0u; i < 6u; i++)
            {
                rx_array[i] = __REV16(rx_array[i]);
            }

            (void)memcpy(&sensor_data, &rx_array[0u], sizeof(sensor_data));
        }
    }
}