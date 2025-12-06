#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "SERVO.h"

typedef struct {
    Servo_t pan;
    Servo_t tilt;
} Gimbal_t;

void Gimbal_Init(void);
void Gimbal_SetAngles(int panDeg, int tiltDeg);
void Gimbal_Center(void);

#endif // GIMBAL_H
