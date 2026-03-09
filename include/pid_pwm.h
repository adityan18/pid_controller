#ifndef PID_PWM
#define PID_PWM

#include "pid_mpu6050.h"

#define SERVO_PWM_TP       20000u /* 20 ms */
#define SERVO_CLOCK_FACTOR 84u    /* 16 MHz / 16 = 1 MHz timer clock, so 1 count = 1 us */

#define SERVO_PWM_MIN 0x280
#define SERVO_PWM_MID 0x5D0
#define SERVO_PWM_MAX 0x960

typedef struct
{
    float    theta;
    float    t_s;
    float    alpha;
    float    k_p;
    float    k_d;
    uint32_t pwm_mid;
    uint32_t pwm_max;
    uint32_t pwm_min;
} pid_controller_params_t;

bool pid_pwm_Init(void);

#endif /* PID_PWM */
