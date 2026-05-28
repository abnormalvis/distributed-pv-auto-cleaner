#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 400Hz PWM: 240MHz / (PSC+1) / (ARR+1) = 240MHz / 100 / 6000 = 400Hz */
#define MOTOR_PWM_PERIOD    5999
#define MOTOR_DEADBAND      15

void motor_init(void);
void motor_update(uint16_t ch1_steer, uint16_t ch2_throttle);
void motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
