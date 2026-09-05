/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Control-mode selection */
typedef enum
{
    MODE_COMMAND_VELOCITY = 0,
    MODE_COMMAND_POSITION,
    MODE_COMMAND_CURRENT
} Mode_t;

typedef enum
{
    STATE_STOP,
    STATE_FINISH,
    STATE_RUNNING,
    STATE_OVERLOAD,
    STATE_STALL,
    STATE_NO_CALIB
} State_t;

extern volatile Mode_t requestMode;
extern volatile Mode_t runningMode;
extern volatile State_t StepperState;

extern volatile int32_t ratedVelocity;
extern volatile int32_t velocityLimit;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void SetVelocitySetPoint(int32_t _vel);
extern void SetCurrentSetPoint(int32_t _cur);

extern int32_t GetPosition();
extern void SetPositionSetPoint(int32_t _pos);

extern bool SetPositionSetPointWithTime(int32_t _pos, float _time);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PB1_Pin GPIO_PIN_0
#define PB1_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define Output_Ap_Pin GPIO_PIN_12
#define Output_Ap_GPIO_Port GPIOB
#define Output_Am_Pin GPIO_PIN_13
#define Output_Am_GPIO_Port GPIOB
#define Output_Bp_Pin GPIO_PIN_14
#define Output_Bp_GPIO_Port GPIOB
#define Output_Bm_Pin GPIO_PIN_15
#define Output_Bm_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS  51200   /* 100 steps x 256 micro-steps (must match CALIB_SUBDIVIDE_STEPS) */
#define CONTROL_FREQUENCY                 20000   /* Hz – must match TIM5 reload period */
#define SOFT_DIVIDE_NUM                   256     /* quarter-circle offset for 90-deg FOC lead */
#define RATED_CURRENT_MA                  1500    /* mA – peak winding current limit */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
