#include "motor.h"
#include "tim.h"
#include "crsf.h"
#include "can_io.h"

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

/* === 6-Channel RC-style control (PWM 1000-2000, center=1500) === */

#define CHANNEL_PWM_CENTER    1500
#define CHANNEL_PWM_HALFRANGE  500   /* max deviation from center */

/**
 * @brief  Convert 6 RC-style channels (PWM 1000-2000) to motor/aux commands.
 *
 * Channel mapping:
 *   [0] Steering  — 1000=left,  1500=center, 2000=right
 *   [1] Throttle  — 1000=rev,   1500=stop,   2000=fwd
 *   [2] Aux 1     — brush motor  (<1500=off, >1500=proportional fwd)
 *   [3] Aux 2     — vacuum       (<1500=off, >1500=proportional fwd)
 *   [4] Aux 3     — lift         (reserved for servo/actuator)
 *   [5] Aux 4     — spray pump   (reserved)
 *
 * Drive motors (ch0+ch1) use mixed differential steering.
 * Aux channels (ch2-5) map to CAN IO relay outputs using a threshold.
 */
void motor_update_from_channels(const int16_t channels[6])
{
    /* ── Drive: differential mixing ── */
    int16_t steer    = channels[0] - CHANNEL_PWM_CENTER;   /* -500 .. +500 */
    int16_t throttle = channels[1] - CHANNEL_PWM_CENTER;   /* -500 .. +500 */

    int16_t left_cmd  = throttle + steer;
    int16_t right_cmd = throttle - steer;

    /* Clamp to half-range */
    if (left_cmd  >  CHANNEL_PWM_HALFRANGE) left_cmd  =  CHANNEL_PWM_HALFRANGE;
    if (left_cmd  < -CHANNEL_PWM_HALFRANGE) left_cmd  = -CHANNEL_PWM_HALFRANGE;
    if (right_cmd >  CHANNEL_PWM_HALFRANGE) right_cmd =  CHANNEL_PWM_HALFRANGE;
    if (right_cmd < -CHANNEL_PWM_HALFRANGE) right_cmd = -CHANNEL_PWM_HALFRANGE;

    /* Scale to PWM: ±500 → ±5999 */
    int16_t left_pwm  = (int16_t)((int32_t)left_cmd  * MOTOR_PWM_PERIOD / CHANNEL_PWM_HALFRANGE);
    int16_t right_pwm = (int16_t)((int32_t)right_cmd * MOTOR_PWM_PERIOD / CHANNEL_PWM_HALFRANGE);

    set_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_1, left_pwm);
    set_motor(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_1, right_pwm);

    /* ── Aux channels → CAN IO relay outputs ── */
    /* Map ch2-ch5 to 4 relay channels using threshold at center.
     * ch > 1500 → relay ON, ch <= 1500 → relay OFF */
    {
        uint8_t relay_mask = 0;
        if (channels[2] > CHANNEL_PWM_CENTER) relay_mask |= (1 << 0);  /* relay 0: brush */
        if (channels[3] > CHANNEL_PWM_CENTER) relay_mask |= (1 << 1);  /* relay 1: vacuum */
        if (channels[4] > CHANNEL_PWM_CENTER) relay_mask |= (1 << 2);  /* relay 2: lift   */
        if (channels[5] > CHANNEL_PWM_CENTER) relay_mask |= (1 << 3);  /* relay 3: spray  */
        can_io_write_relay(relay_mask);
    }
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
