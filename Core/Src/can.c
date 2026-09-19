/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "board_config.h"
#include "encoder_calib.h"
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef sFilterConfig;
  //filter one (stack light blink)
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
	  /* Filter configuration Error */
	  Error_Handler();
  }

  HAL_CAN_Start(&hcan); //start CAN

  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

/* Configure Transmission process */
  TxHeader.StdId = ThisCAN_NodeId;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
{
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
	{
		if (HAL_CAN_AddTxMessage(&hcan, pHeader, data, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* CanHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    uint8_t id = (RxHeader.StdId >> 7); // 4Bits ID & 7Bits Msg
    uint8_t cmd = RxHeader.StdId & 0x7F; // 4Bits ID & 7Bits Msg
    if (id == 0 || id == ThisCAN_NodeId)
    {
        OnCanCmd(cmd, RxData, RxHeader.DLC);
    }
}

void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len)
{
	int32_t tmpV = 0;
    switch (_cmd)
    {
    	case 0x01:
    	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      break;

      case 0x02: // Do Calibration
    	  calibState = CALIB_START;
        if (_data[4] == 1) // Ack Calibration Done
        {
          txHeader.StdId = (TargetCAN_NodeId << 7) | 0x02; // 0x02 is Do Calibration
          TxData[0] = calibState == CALIB_DONE ? 1u : 0u;
          CAN_Send(&txHeader, TxData);
        }
      break;

      case 0x03: // Set Current SetPoint
        if (runningMode != MODE_COMMAND_CURRENT)
        {
          requestMode = MODE_COMMAND_CURRENT;
        }
        memcpy(&tmpV, _data, sizeof(int32_t));
        SetCurrentSetPoint(tmpV);
      break;

      case 0x04:  // Set Velocity SetPoint
        if (runningMode != MODE_COMMAND_VELOCITY)
        {
          requestMode = MODE_COMMAND_VELOCITY;
        }
        memcpy(&tmpV, _data, sizeof(tmpV));
        SetVelocitySetPoint(tmpV);
      break;

      case 0x05:  // Set Position SetPoint
        if (runningMode != MODE_COMMAND_POSITION)
        {
          ratedVelocity = boardConfig.velocityLimit;
          requestMode = MODE_COMMAND_POSITION;
        }
        memcpy(&tmpV, _data, sizeof(tmpV));
        SetPositionSetPoint( tmpV );
        if (_data[4] == 1) // Need Position & Finished ACK
        {
          int32_t TxCanBuf = GetPosition();
          memcpy(TxData, &TxCanBuf, sizeof(TxCanBuf));

          TxData[4] = StepperState == STATE_FINISH ? 1 : 0;
          TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x23; // 0x23 is GetPosition, also.
          CAN_Send(&TxHeader, TxData);
        }
		  break;

    	case 0x06:  // Set Position with Time - currently not used
        if (runningMode != MODE_COMMAND_POSITION)
        {
          requestMode = MODE_COMMAND_POSITION;
        }
        float tmpTime = 0;
        memcpy(&tmpV, _data, sizeof(tmpV));
        memcpy(&tmpTime, _data + 4, sizeof(tmpTime));

        SetPositionSetPointWithTime(tmpV, (float) tmpTime);
		  break;

    	case 0x07: // Set Position with Time and Velocity Limit
        if (runningMode != MODE_COMMAND_POSITION)
        {
          requestMode = MODE_COMMAND_POSITION;
        }
        // Set VelocityLimit (boardConfig.velocityLimit)
        memcpy(&boardConfig.velocityLimit, _data + 4, sizeof(int32_t));
        ratedVelocity = boardConfig.velocityLimit;
        // Set SetPositionSetPoint
        int32_t tempPosition = 0;
        memcpy(&tempPosition, _data, sizeof(tempPosition));
        SetPositionSetPoint(tempPosition);

        // Sending current position to master
        int32_t TxCanBuf = GetPosition();
        memcpy(TxData, &TxCanBuf, sizeof(TxCanBuf));
        TxData[4] = StepperState == STATE_FINISH ? 1 : 0;
        TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x23; // 0x23 is GetPosition, also.
        CAN_Send(&TxHeader, TxData);
		  break;

      /* 0x10~0x1F CMDs with Memory */
      case 0x11:  // Set Node-ID and Store to Flash
        memcpy(boardConfig.canNodeId, _data, sizeof(uint32_t));
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x12:  // Set Current-Limit and Store to Flash
        memcpy(&boardConfig.currentLimit, _data, sizeof(int32_t));
        ratedCurrent = boardConfig.currentLimit;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x13:  // Set Velocity-Limit and Store to Flash
        memcpy(&boardConfig.velocityLimit, _data, sizeof(int32_t));
        ratedVelocity = boardConfig.velocityLimit;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x14:  // Set Acceleration （and Store to Flash）
        memcpy(&boardConfig.velocityAcc, _data, sizeof(int32_t));
        velocityAcc = boardConfig.velocityAcc;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x15:  // Apply Home-Position and Store to Flash
        memcpy(&boardConfig.encoderHomeOffset, _data, sizeof(int32_t));
        encoderHomeOffset = boardConfig.encoderHomeOffset;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x16:  // Set Auto-Enable and Store to Flash
        // Not yet implemented.
      break;

      case 0x17:  // Set DCE Kp
        memcpy(&dce.kp, _data, sizeof(int32_t));
        boardConfig.dce_kp = dce.kp;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x18:  // Set DCE Kv
        memcpy(&dce.kv, _data, sizeof(int32_t));
        boardConfig.dce_kv = dce.kv;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x19:  // Set DCE Ki
        memcpy(&dce.ki, _data, sizeof(int32_t));
        boardConfig.dce_ki = dce.ki;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x1A:  // Set DCE Kd
        memcpy(&dce.kd, _data, sizeof(int32_t));
        boardConfig.dce_kd = dce.kd;
        if (_data[4] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x1B:  // Set Enable Stall-Protect
        // Not yet implemented. Need to implement stall-protect in the main loop.
      break;

      case 0x1C:  // Set PID gain
        memcpy(&pid.kp, _data, sizeof(int16_t));
        memcpy(&pid.ki, _data + 2, sizeof(int16_t));
        memcpy(&pid.kd, _data + 4, sizeof(int16_t));
        boardConfig.pid_kp = pid.kp;
        boardConfig.pid_ki = pid.ki;
        boardConfig.pid_kd = pid.kd;
        if (_data[6] == 1) boardConfig.configStatus = CONFIG_COMMIT;
      break;

      case 0x21: // Get Current
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x21;
        int32_t TxCanBuf = GetFocCurrent();
        memcpy(TxData, &TxCanBuf, sizeof(int32_t));
        TxData[4] = StepperState == STATE_FINISH ? 1 : 0;
        CAN_Send(&TxHeader, TxData);
      break;

      case 0x22: // Get Velocity
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x22;
        float TxCanBuf = GetVelocity();
        memcpy(TxData, &TxCanBuf, sizeof(float));
        TxData[4] = StepperState == STATE_FINISH ? 1 : 0;
        CAN_Send(&TxHeader, TxData);
      break;

      case 0x23: // Get Position
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
        int32_t TxCanBuf = GetPosition();
        memcpy(TxData, &TxCanBuf, sizeof(int32_t));
        TxData[4] = StepperState == STATE_FINISH ? 1 : 0;
        CAN_Send(&TxHeader, TxData);
      break;

      case 0x24: // Get Offset
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x24;
        int32_t TxCanBuf = encoderHomeOffset;
        memcpy(TxData, &TxCanBuf, sizeof(int32_t));
        CAN_Send(&TxHeader, TxData);
      break;

      case 0x25: // Get temperature
        // Not yet implemented
      break;

      case 0x7D:  // Undo Configs
         boardConfig.configStatus = CONFIG_RESTORE;
      break;
      case 0x7E:  // Default Configs
          boardConfig.configStatus = CONFIG_DEFAULT;
      break;
      case 0x7F:  // Reboot
          HAL_NVIC_SystemReset();
      break;

    	default:
    		break;
    }
}
/* USER CODE END 1 */
