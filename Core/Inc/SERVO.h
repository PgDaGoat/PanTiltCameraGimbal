#ifndef SERVO_H
#define SERVO_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t minPulseUs;   // e.g. 1000
    uint16_t maxPulseUs;   // e.g. 2000
} Servo_t;

void Servo_Init(Servo_t *servo,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                uint16_t minPulseUs,
                uint16_t maxPulseUs);

void Servo_WriteUs(Servo_t *servo, uint16_t pulseUs);
void Servo_WriteAngle(Servo_t *servo, float angleDeg);

#endif // SERVO_H
