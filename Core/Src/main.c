/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body â€“ Pan/Tilt with DEBUG LED feedback
  ******************************************************************************
  * LED FEEDBACK CODES:
  * - 3 quick blinks at startup = System initialized OK
  * - Slow blink = Running normally
  * - LED ON when USART2 receives data
  * - Fast blink = Error
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gimbal.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;   // Bluetooth
UART_HandleTypeDef huart2;   // PC / YOLO

/* USER CODE BEGIN PV */

/* ---------- Shared gimbal state ---------- */
static float panAngle  = 90.0f;
static float tiltAngle = 90.0f;
static volatile float targetPan  = 90.0f;
static volatile float targetTilt = 90.0f;
static volatile uint8_t pc_cmd_pending = 0;
static volatile uint8_t bt_cmd_pending = 0;


/* Debug counters */
static uint32_t uart1_rx_count = 0;
static uint32_t uart2_rx_count = 0;
static uint32_t commands_processed = 0;

/* ---------- Helper: clamp servo angles ---------- */
static void ClampAngles(void)
{
    if (panAngle  < 0.0f)   panAngle  = 0.0f;
    if (panAngle  > 180.0f) panAngle  = 180.0f;
    if (tiltAngle < 0.0f)   tiltAngle = 0.0f;
    if (tiltAngle > 180.0f) tiltAngle = 180.0f;
}

/* =========================================================
 *                 BLUETOOTH (USART1, MicroBlue)
 * ========================================================= */

#define BT_ID_BUF_SIZE    16
#define BT_VAL_BUF_SIZE   16

static uint8_t bt_rx_byte;
static char    bt_idBuf[BT_ID_BUF_SIZE];
static char    bt_valBuf[BT_VAL_BUF_SIZE];
static uint8_t bt_idIndex  = 0;
static uint8_t bt_valIndex = 0;

typedef enum {
    BT_WAIT_SOH,
    BT_READ_ID,
    BT_READ_VALUE
} BT_ParseState_t;

static BT_ParseState_t btState = BT_WAIT_SOH;

#define PAN_STEP_DEG   3.0f
#define TILT_STEP_DEG  3.0f

static void BT_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
}

static void BT_HandleFrame(const char *id, const char *value)
{
    if (strcmp(value, "1") != 0) {
        return;
    }

    if (strcmp(id, "DL") == 0) {
        panAngle -= PAN_STEP_DEG;
    } else if (strcmp(id, "DR") == 0) {
        panAngle += PAN_STEP_DEG;
    } else if (strcmp(id, "DU") == 0) {
        tiltAngle -= TILT_STEP_DEG;
    } else if (strcmp(id, "DD") == 0) {
        tiltAngle += TILT_STEP_DEG;
    } else if (strcmp(id, "DC") == 0) {
        panAngle  = 90.0f;
        tiltAngle = 90.0f;
    }

    ClampAngles();

    // store into targets and set flag â€“ main loop will move the servos
    targetPan  = panAngle;
    targetTilt = tiltAngle;
    bt_cmd_pending = 1;

    commands_processed++;
}


static void BT_ProcessByte(uint8_t byte)
{
    switch (btState)
    {
    case BT_WAIT_SOH:
        if (byte == 0x01) {
            bt_idIndex  = 0;
            bt_valIndex = 0;
            btState     = BT_READ_ID;
        }
        break;

    case BT_READ_ID:
        if (byte == 0x02) {
            if (bt_idIndex < BT_ID_BUF_SIZE)
                bt_idBuf[bt_idIndex] = '\0';
            btState = BT_READ_VALUE;
        } else {
            if (bt_idIndex < BT_ID_BUF_SIZE - 1) {
                bt_idBuf[bt_idIndex++] = (char)byte;
            }
        }
        break;

    case BT_READ_VALUE:
        if (byte == 0x03) {
            if (bt_valIndex < BT_VAL_BUF_SIZE)
                bt_valBuf[bt_valIndex] = '\0';

            BT_HandleFrame(bt_idBuf, bt_valBuf);
            btState = BT_WAIT_SOH;
        } else {
            if (bt_valIndex < BT_VAL_BUF_SIZE - 1) {
                bt_valBuf[bt_valIndex++] = (char)byte;
            }
        }
        break;

    default:
        btState = BT_WAIT_SOH;
        break;
    }
}

/* =========================================================
 *                PC / YOLO SERIAL (USART2)
 * ========================================================= */

#define PC_LINE_BUF_SIZE  32

static uint8_t pc_rx_byte;
static char    pc_line_buf[PC_LINE_BUF_SIZE];
static uint8_t pc_line_idx = 0;

static void PC_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
}

static void PC_HandleLine(const char *line)
{
    int panInt = 0, tiltInt = 0;
    int count;

    // Try "PAN,TILT"
    count = sscanf(line, "%d,%d", &panInt, &tiltInt);

    // Also accept "PAN TILT" (space separated)
    if (count != 2)
    {
        count = sscanf(line, "%d %d", &panInt, &tiltInt);
    }

    // If only one number, treat it as pan only; keep tiltAngle as-is
    if (count == 1)
    {
        float p = (float)panInt;

        if (p < 0.0f)   p = 0.0f;
        if (p > 180.0f) p = 180.0f;

        targetPan  = p;
        targetTilt = tiltAngle;   // donâ€™t change tilt
        pc_cmd_pending = 1;
        commands_processed++;

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        return;
    }

    // If we got both pan and tilt:
    if (count == 2)
    {
        float p = (float)panInt;
        float t = (float)tiltInt;

        if (p < 0.0f)   p = 0.0f;
        if (p > 180.0f) p = 180.0f;
        if (t < 0.0f)   t = 0.0f;
        if (t > 180.0f) t = 180.0f;

        targetPan  = p;
        targetTilt = t;
        pc_cmd_pending = 1;
        commands_processed++;

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }

    // If count == 0, do nothing (bad line)
}





static void PC_ProcessByte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (pc_line_idx > 0)
        {
            pc_line_buf[pc_line_idx] = '\0';
            PC_HandleLine(pc_line_buf);
            pc_line_idx = 0;
        }
    }
    else
    {
        if (pc_line_idx < PC_LINE_BUF_SIZE - 1)
        {
            pc_line_buf[pc_line_idx++] = (char)byte;
        }
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN 0 */

/* Global UART RX callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_rx_count++;
        BT_ProcessByte(bt_rx_byte);
        BT_StartReceive();
    }
    else if (huart->Instance == USART2)
    {
        uart2_rx_count++;
        PC_ProcessByte(pc_rx_byte);
        PC_StartReceive();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();  // Bluetooth
  MX_USART2_UART_Init();  // PC/YOLO

  /* USER CODE BEGIN 2 */

  // Startup blink sequence - 3 quick blinks
  for (int i = 0; i < 3; i++)
  {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_Delay(200);
  }

 HAL_Delay(500);
  // Initialize gimbal (servos)
  Gimbal_Init();
  Gimbal_Center();   // 90Â°, 90Â°

  HAL_Delay(500);

  // *** CRITICAL: Start UART interrupt reception ***
  HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);

  // Send a test message via USART2 to confirm TX works
  char startup_msg[] = "STM32 Ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startup_msg, strlen(startup_msg), 100);

  /* USER CODE END 2 */

  // LED pattern to show which mode
  // 2 long blinks = waiting for commands
  for (int i = 0; i < 2; i++)
  {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_Delay(500);
  }

  /* Infinite loop */
  uint32_t last_blink = 0;
  while (1)
  {
      // Slow heartbeat blink (every 1 second)
      if (HAL_GetTick() - last_blink > 1000)
      {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          last_blink = HAL_GetTick();
      }

      // Apply any pending command from PC or BT
      if (pc_cmd_pending || bt_cmd_pending)
      {
          __disable_irq();
          pc_cmd_pending = 0;
          bt_cmd_pending = 0;
          float p = targetPan;
          float t = targetTilt;
          __enable_irq();

          panAngle  = p;
          tiltAngle = t;
          //ClampAngles();
          Gimbal_SetAngles(panAngle, tiltAngle);
      }

      // Optional: Send debug info every 5 seconds
      static uint32_t last_debug = 0;
      if (HAL_GetTick() - last_debug > 5000)
      {
          char debug[64];
          sprintf(debug, "U1:%lu U2:%lu CMD:%lu\r\n",
                  uart1_rx_count, uart2_rx_count, commands_processed);
          HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 100);
          last_debug = HAL_GetTick();
      }

      HAL_Delay(10);
  }
}

/* ================== System Configuration ================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;    // FIXED
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;     // FIXED
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;  // Bluetooth
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;  // PC/YOLO
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for(volatile int i = 0; i < 100000; i++);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
