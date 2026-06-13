#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 400Hz PWM: 240MHz / (PSC+1) / (ARR+1) = 240MHz / 100 / 6000 = 400Hz */
#define MOTOR_PWM_PERIOD     5999
#define MOTOR_DEADBAND         15
#define SERIAL_SPEED_MAX    32767   /* int16 full-scale for ROS2 serial speed */

void motor_init(void);
void motor_update(uint16_t ch1_steer, uint16_t ch2_throttle);
void motor_update_raw(int16_t left_speed, int16_t right_speed);
void motor_update_from_channels(const int16_t channels[6]);
void motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
