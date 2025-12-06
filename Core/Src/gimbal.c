#include "gimbal.h"

extern TIM_HandleTypeDef htim2;   // from main.c / CubeMX

static Gimbal_t gimbal;

// conservative safe limits for most servos
#define SERVO_MIN_US  1000U
#define SERVO_MAX_US  2000U

void Gimbal_Init(void)
{
    // PA0 -> TIM2_CH1 (Pan)
    Servo_Init(&gimbal.pan,  &htim2, TIM_CHANNEL_1, SERVO_MIN_US, SERVO_MAX_US);

    // PA1 -> TIM2_CH2 (Tilt)
    Servo_Init(&gimbal.tilt, &htim2, TIM_CHANNEL_2, SERVO_MIN_US, SERVO_MAX_US);

    //Gimbal_Center();
}

void Gimbal_SetAngles(int panDeg, int tiltDeg)
{
    Servo_WriteAngle(&gimbal.pan,  panDeg);
    Servo_WriteAngle(&gimbal.tilt, tiltDeg);
}

void Gimbal_Center(void)
{
    Gimbal_SetAngles(90, 90);
}
