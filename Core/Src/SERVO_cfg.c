#include "SERVO.h"

// Convert microseconds -> timer ticks assuming 20 ms frame (50 Hz)
static uint32_t Servo_PulseUsToTicks(Servo_t *servo, uint16_t pulseUs)
{
    uint32_t periodTicks = __HAL_TIM_GET_AUTORELOAD(servo->htim) + 1U; // ARR+1
    // 20 000 µs per frame at 50 Hz
    uint32_t ticks = (uint32_t)pulseUs * periodTicks / 20000U;
    if (ticks > periodTicks) {
        ticks = periodTicks;
    }
    return ticks;
}

void Servo_Init(Servo_t *servo,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                uint16_t minPulseUs,
                uint16_t maxPulseUs)
{
    servo->htim       = htim;
    servo->channel    = channel;
    servo->minPulseUs = minPulseUs;
    servo->maxPulseUs = maxPulseUs;

    // Start PWM on this channel
    HAL_TIM_PWM_Start(servo->htim, servo->channel);

    // Go to center on power-up
    //Servo_WriteAngle(servo, 90.0f);
}

void Servo_WriteUs(Servo_t *servo, uint16_t pulseUs)
{
    if (pulseUs < servo->minPulseUs) pulseUs = servo->minPulseUs;
    if (pulseUs > servo->maxPulseUs) pulseUs = servo->maxPulseUs;

    uint32_t periodTicks = __HAL_TIM_GET_AUTORELOAD(servo->htim) + 1U;
    uint32_t ccr = (uint32_t)pulseUs * periodTicks / 20000U;

    if (ccr > periodTicks) ccr = periodTicks;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, ccr);
}

void Servo_WriteAngle(Servo_t *servo, float angleDeg)
{
    if (angleDeg < 0.0f)   angleDeg = 0.0f;
    if (angleDeg > 180.0f) angleDeg = 180.0f;

    float span = (float)(servo->maxPulseUs - servo->minPulseUs);
    uint16_t pulse = (uint16_t)(servo->minPulseUs + (angleDeg / 180.0f) * span);

    Servo_WriteUs(servo, pulse);
}
