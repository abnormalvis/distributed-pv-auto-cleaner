#include "motor.h"
#include "tim.h"
#include "crsf.h"

static void set_motor(TIM_HandleTypeDef *htim, uint32_t ch_fwd, uint32_t ch_bwd, int16_t speed)
{
    if (speed > MOTOR_PWM_PERIOD) speed = MOTOR_PWM_PERIOD;

    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(htim, ch_fwd, speed);
        __HAL_TIM_SET_COMPARE(htim, ch_bwd, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(htim, ch_fwd, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_bwd, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(htim, ch_fwd, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_bwd, 0);
    }
}

void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

    HAL_GPIO_WritePin(Motor1_EN_GPIO_Port, Motor1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor2_EN_GPIO_Port, Motor2_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN1_24V_GPIO_Port, EN1_24V_Pin, GPIO_PIN_SET);
}

void motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

    HAL_GPIO_WritePin(Motor1_EN_GPIO_Port, Motor1_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor2_EN_GPIO_Port, Motor2_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN1_24V_GPIO_Port, EN1_24V_Pin, GPIO_PIN_RESET);
}

void motor_update_raw(int16_t left_speed, int16_t right_speed)
{
    /* Linear mapping: int16 [-32768, 32767] -> PWM [-5999, 5999] */
    int16_t left_pwm  = (int16_t)((int32_t)left_speed  * MOTOR_PWM_PERIOD / SERIAL_SPEED_MAX);
    int16_t right_pwm = (int16_t)((int32_t)right_speed * MOTOR_PWM_PERIOD / SERIAL_SPEED_MAX);

    set_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_1, left_pwm);
    set_motor(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_1, right_pwm);
}

void motor_update(uint16_t ch1_steer, uint16_t ch2_throttle)
{
    int16_t steer    = (int16_t)ch1_steer - CRSF_CHANNEL_MID;
    int16_t throttle = (int16_t)ch2_throttle - CRSF_CHANNEL_MID;

    if (steer < MOTOR_DEADBAND && steer > -MOTOR_DEADBAND) steer = 0;
    if (throttle < MOTOR_DEADBAND && throttle > -MOTOR_DEADBAND) throttle = 0;

    int16_t left_cmd  = throttle + steer;
    int16_t right_cmd = throttle - steer;

    int16_t limit = CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID;
    if (left_cmd > limit)  left_cmd = limit;
    if (left_cmd < -limit) left_cmd = -limit;
    if (right_cmd > limit)  right_cmd = limit;
    if (right_cmd < -limit) right_cmd = -limit;

    int16_t left_pwm  = (int32_t)left_cmd  * MOTOR_PWM_PERIOD / limit;
    int16_t right_pwm = (int32_t)right_cmd * MOTOR_PWM_PERIOD / limit;

    set_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_1, left_pwm);
    set_motor(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_1, right_pwm);
}
