#include "pid_pwm.h"
#include "pid_common.h"
#include "pid_mpu6050.h"

#include <math.h>

#define M_PI 3.14159265358979323846

/************************ D A T A ************************/

extern osMessageQueueId_t sensor_to_actuator_raw_queue;       /* message queue id */
extern osMessageQueueId_t sensor_to_actuator_processed_queue; /* event logger queue */

osThreadId_t mpu6050_sample_sensor_data_thread_id;

pid_controller_params_t pid_params = {
    .theta = 0.0f,
    .t_s = 0.1f,
    .alpha = 0.5f,
    .k_p = 15.0f,
    .k_d = 0.1f,
    .pwm_mid = SERVO_PWM_MID,
    .pwm_max = SERVO_PWM_MAX,
    .pwm_min = SERVO_PWM_MIN,
};

/************************ P R O T O T Y P E ************************/
static void pid_pwm_GenerateThread(void *arg);
static uint32_t pid_pwm_GeneratePwmSignal(pid_controller_params_t *params, mpu6050_processed_data_t *processed_data);
static void pwm_Initialize(void);
static void pwm_SetPwmSignal(uint32_t pwm_signal);

/************************ P U B L I C  F U N C T I O N S ************************/
bool pid_pwm_Init(void)
{
    bool thread_creation_successful = false;

    pwm_Initialize();

    osThreadAttr_t thread_attr = {
        .name = "pid_pwm_generate_thread",
        .priority = osPriorityNormal,
        .stack_size = 512 * 4,
    };

    mpu6050_sample_sensor_data_thread_id = osThreadNew(pid_pwm_GenerateThread, NULL, &thread_attr);

    return true;
}

/************************ P R I V A T E  F U N C T I O N S ************************/
static void pwm_Initialize(void)
{
    /* Enable RCC to GPIOA and TIM3 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure GPIO, PA6 as PWM output */
    GPIO_InitTypeDef gpio_a_config = {.Pin = GPIO_PIN_6,
                                      .Mode = GPIO_MODE_AF_PP,
                                      .Pull = GPIO_NOPULL,
                                      .Speed = GPIO_SPEED_FREQ_LOW,
                                      .Alternate = GPIO_AF2_TIM3};

    HAL_GPIO_Init(GPIOA, &gpio_a_config);

    /* Configure Timer Clock, Prescalar and Auto Reload Register */
    TIM3->PSC = SERVO_CLOCK_FACTOR - 1u;
    TIM3->ARR = SERVO_PWM_TP - 1u;
    TIM3->CCR1 = SERVO_PWM_MID; /* Initialize with mid position */

    /* Configure PWM mode */
    TIM3->CCMR1 |= (6 << 4);
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;

    /* Initialize with mid position */
    TIM3->CCR1 = SERVO_PWM_MID;
}

static void pid_pwm_GenerateThread(void *arg)
{
    (void)arg;

    while (1)
    {
        pid_sensor_data_queue_t raw_queue_data;
        osStatus_t status = osMessageQueueGet(sensor_to_actuator_raw_queue, &raw_queue_data, NULL, osWaitForever);

        if (status == osOK)
        {
            /* Process the raw sensor data to get acceleration and gyroscope values */
            mpu6050_processed_data_t processed_data = pid_mpu6050_ProcessRawSensorData(raw_queue_data.sensor_data);
            pid_sensor_data_queue_t processed_queue_data = {
                .index = raw_queue_data.index,
                .processed_sensor_data = processed_data,
            };
            (void)osMessageQueuePut(sensor_to_actuator_processed_queue, &processed_queue_data, NULL, 0u);
            /* Generate PWM signal based on the processed sensor data */
           uint32_t pwm_signal = pid_pwm_GeneratePwmSignal(&pid_params, &processed_data);
           pwm_SetPwmSignal(pwm_signal);
        }
    }
}

static uint32_t pid_pwm_GeneratePwmSignal(pid_controller_params_t *params, mpu6050_processed_data_t *processed_data)
{
    float theta_acc = atan2f(processed_data->ay, processed_data->az) * (180.0f / M_PI);

    params->theta =
        params->alpha * (params->theta + processed_data->gx * params->t_s) + (1.0f - params->alpha) * theta_acc;

    float error = ((params->k_p * params->theta) + (params->k_d * (processed_data->gx)));

    float pwm_signal = params->pwm_mid + error;

    if (pwm_signal > params->pwm_max)
    {
        pwm_signal = params->pwm_max;
    }
    else if (pwm_signal < params->pwm_min)
    {
        pwm_signal = params->pwm_min;
    }

    return (uint32_t)pwm_signal;
}

static void pwm_SetPwmSignal(uint32_t pwm_signal)
{
    TIM3->CCR1 = pwm_signal;
}