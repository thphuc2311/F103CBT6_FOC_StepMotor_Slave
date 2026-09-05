/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "Driver_tb67h450.h"
#include "Encoder_mt6701.h"
#include "encoder_calib.h"
#include "board_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS  51200   /* 100 steps x 256 micro-steps (must match CALIB_SUBDIVIDE_STEPS) */
#define CONTROL_FREQUENCY                 20000   /* Hz – must match TIM5 reload period */
#define SOFT_DIVIDE_NUM                   256     /* quarter-circle offset for 90-deg FOC lead */
#define RATED_CURRENT_MA                  1500    /* mA – peak winding current limit */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Encoder raw reading */
volatile uint16_t angleData = 0;

/* Position tracking */
volatile int32_t realLapPosition = 0, realLapPositionLast = 0;
volatile int32_t realPosition    = 0, realPositionLast    = 0;

/* Velocity estimation (integer IIR) */
volatile int32_t estVelocityIntegral = 0, estVelocity = 0;

/* Lead-angle compensation -> estimated rotor position */
volatile int32_t estLeadPosition = 0, estPosition = 0;

/* Control set-points */
volatile int32_t goalVelocity = 0;   /* desired velocity [subdivisions/s], set from main loop */
volatile int32_t softVelocity = 0;   /* ramped velocity fed to PID */
int32_t          UserReq_Vel = 0;    /* User request Goal Velocity via Debugger */

/* FOC output */
volatile int32_t focCurrent = 0, focPosition = 0;

/* Motion-planner state (velocity ramp) */
volatile int32_t velocityIntegral = 0, trackVelocity = 0, goVelocity = 0;
volatile int32_t velocityAcc      = 0;   /* acceleration step [subdivisions/s per tick] */
volatile int32_t ratedVelocity    = 0;   /* maximum velocity  [subdivisions/s]           */
uint16_t angleData_Calibrated = 0;

/* Driver debug symbols – declared extern in Driver_tb67h450.c */
int16_t SinMapValues_PhaseA_debug1 = 0, SinMapValues_PhaseB_debug1 = 0;
int16_t SinMapValues_PhaseA_debug2 = 0, SinMapValues_PhaseB_debug2 = 0;

int32_t a,b;

/* PID controller state (velocity loop) */
typedef struct
{
    int32_t kp, ki, kd;
    int32_t vError, vErrorLast;
    int32_t outputKp, outputKi, outputKd;
    int32_t integralRound, integralRemainder;
    int32_t output;
} PID_t;

volatile PID_t pid;

/* DCE controller state (position loop) */
typedef struct
{
    int32_t kp, kv, ki, kd;
    int32_t pError, vError;
    int32_t outputKp, outputKi, outputKd;
    int32_t integralRound, integralRemainder;
    int32_t output;
} DCE_t;

volatile DCE_t dce;

volatile Mode_t requestMode  = MODE_COMMAND_VELOCITY;   /* set from main loop  */
volatile Mode_t runningMode  = MODE_COMMAND_VELOCITY;   /* currently active    */
volatile State_t StepperState = STATE_STOP;
volatile bool   softNewCurve = false;                   /* seed tracker on mode change */

/* Position set-points */
volatile int32_t goalPosition = 0;   /* desired position [subdivisions], set from main loop */
volatile int32_t softPosition = 0;   /* ramped position fed to DCE                          */
int32_t 	UserReq_Pos 	= 51200; /* User request Goal Position via Debugger */
float 		UserReq_Time 	= 1.0f;  /* User request Goal Time to Position via Debugger */

/* Current set-points (MODE_COMMAND_CURRENT) */
volatile int32_t goalCurrent = 0;    /* desired current [mA], set from main loop            */
volatile int32_t softCurrent = 0;    /* ramped current fed to FOC                           */
int32_t 		 UserReq_Cur = 0;	 /* User request Goal Current via Debugger*/

/* Current motion-planner state (current ramp) */
volatile int32_t currentIntegral = 0, trackCurrent = 0, goCurrent = 0;
volatile int32_t currentAcc      = 0;   /* current ramp step [mA/s]                          */
volatile int32_t ratedCurrentAcc = 1 * 1000;     /* (mA/s) */

/* Motion limits / home reference */
volatile int32_t encoderHomeOffset = 0;   /* zero-position offset [subdivisions]             */
volatile int32_t velocityLimit     = 0;   /* absolute max cruise velocity [subdivisions/s]   */

/* Position motion-planner state (trapezoidal position tracker) */
volatile int32_t posVelocityUpAcc      = 0, posVelocityDownAcc = 0;
volatile int32_t posSpeedLockingBrake  = 0;
float            posQuickVelocityDownAcc = 0.0f;
volatile int32_t posVelocityIntegral   = 0, posTrackVelocity = 0;
volatile int32_t posPositionIntegral   = 0, posTrackPosition = 0;
volatile int32_t posGoLocation         = 0, posGoVelocity    = 0;

/* Define CAN header for TxCAN */
//CAN_TxHeaderTypeDef txHeader =
//{
//	.StdId = 0x01,
//	.ExtId = 0x00,
//	.IDE = CAN_ID_STD,
//	.RTR = CAN_RTR_DATA,
//	.DLC = 8,
//	.TransmitGlobalTime = DISABLE
//};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t GetPosition ()
{
	return (realPosition - encoderHomeOffset);
}

/*
 * CalcVelocityIntegral
 * Integrates _velocity into trackVelocity using a sub-step accumulator so
 * fractional increments are preserved at 20 kHz.
 */
static void CalcVelocityIntegral(int32_t _velocity)
{
    velocityIntegral += _velocity;
    trackVelocity    += velocityIntegral / CONTROL_FREQUENCY;
    velocityIntegral  = velocityIntegral % CONTROL_FREQUENCY;
}

/*
 * CalcSoftGoal  (velocity ramp generator – MODE_PWM_VELOCITY)
 * Smoothly accelerates / decelerates trackVelocity toward _goalVelocity
 * using the configured velocityAcc step.
 */
static void CalcSoftGoal(int32_t _goalVelocity)
{
    int32_t deltaVelocity = _goalVelocity - trackVelocity;

    if (deltaVelocity == 0)
    {
        trackVelocity = _goalVelocity;
    }
    else if (deltaVelocity > 0)
    {
        if (trackVelocity >= 0)
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity    = _goalVelocity;
            }
        }
        else
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity    = 0;
            }
        }
    }
    else   /* deltaVelocity < 0 */
    {
        if (trackVelocity <= 0)
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity    = _goalVelocity;
            }
        }
        else
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity    = 0;
            }
        }
    }

    goVelocity = trackVelocity;
}

/*
 * CompensateAdvancedAngle
 * Returns an advance offset in subdivisions to compensate rotor phase lag
 * at speed. Coefficients from the DPS-encoder reference design; re-measure
 * when changing sensor type.
 */
static int32_t CompensateAdvancedAngle(int32_t _vel)
{
    int32_t compensate;

    if (_vel < 0)
    {
        if      (_vel > -100000)  compensate = 0;
        else if (_vel > -1300000) compensate = (((_vel + 100000)  * 262) >> 20);
        else if (_vel > -2200000) compensate = (((_vel + 1300000) * 105) >> 20) - 300;
        else                      compensate = (((_vel + 2200000) *  52) >> 20) - 390;

        if (compensate < -430) compensate = -430;
    }
    else
    {
        if      (_vel < 100000)  compensate = 0;
        else if (_vel < 1300000) compensate = (((_vel - 100000)  * 262) >> 20);
        else if (_vel < 2200000) compensate = (((_vel - 1300000) * 105) >> 20) + 300;
        else                     compensate = (((_vel - 2200000) *  52) >> 20) + 390;

        if (compensate > 430) compensate = 430;
    }

    return compensate;
}

/*
 * CalcCurrentToOutput
 * Maps the desired current magnitude to a FOC vector 90 deg ahead (positive
 * current) or behind (negative current) the estimated rotor position, then
 * drives the TB67H450 via SetFocCurrentVector.
 */
static void CalcCurrentToOutput(int32_t current)
{
    focCurrent = current;

    if      (focCurrent > 0) focPosition = estPosition + SOFT_DIVIDE_NUM;  /* lead  90 deg */
    else if (focCurrent < 0) focPosition = estPosition - SOFT_DIVIDE_NUM;  /* lag   90 deg */
    else                     focPosition = estPosition;

    SetFocCurrentVector((uint32_t)focPosition, focCurrent);
}

/*
 * Current-tracker sub-step accumulator (preserves fractional increments at 20 kHz).
 */
static void CurrentTracker_CalcCurrentIntegral(int32_t _current)
{
    currentIntegral += _current;
    trackCurrent    += currentIntegral / CONTROL_FREQUENCY;
    currentIntegral  = currentIntegral % CONTROL_FREQUENCY;
}

/*
 * CurrentTracker_SetCurrentAcc
 * Sets the current ramp step [mA/s] used by the MODE_COMMAND_CURRENT planner.
 * Only use in Init phase
 */
static void CurrentTracker_SetCurrentAcc(int32_t _currentAcc)
{
    currentAcc = _currentAcc;
}

/*
 * CurrentTracker_NewTask
 * Seeds the tracker from the present FOC current so the ramp starts smoothly
 * (called when entering current mode).
 */
static void CurrentTracker_NewTask(int32_t _realCurrent)
{
    currentIntegral = 0;
    trackCurrent    = _realCurrent;
}

/*
 * CurrentTracker_CalcSoftGoal  (current ramp generator – MODE_COMMAND_CURRENT)
 * Smoothly ramps trackCurrent toward _goalCurrent using currentAcc.
 * Direct port of the Ctrl-Step-Driver CurrentTracker::CalcSoftGoal().
 */
static void CurrentTracker_CalcSoftGoal(int32_t _goalCurrent)
{
    int32_t deltaCurrent = _goalCurrent - trackCurrent;

    if (deltaCurrent == 0)
    {
        trackCurrent = _goalCurrent;
    }
    else if (deltaCurrent > 0)
    {
        if (trackCurrent >= 0)
        {
            CurrentTracker_CalcCurrentIntegral(currentAcc);
            if (trackCurrent >= _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent    = _goalCurrent;
            }
        }
        else
        {
            CurrentTracker_CalcCurrentIntegral(currentAcc);
            if (trackCurrent >= 0)
            {
                currentIntegral = 0;
                trackCurrent    = 0;
            }
        }
    }
    else   /* deltaCurrent < 0 */
    {
        if (trackCurrent <= 0)
        {
            CurrentTracker_CalcCurrentIntegral(-currentAcc);
            if (trackCurrent <= _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent    = _goalCurrent;
            }
        }
        else
        {
            CurrentTracker_CalcCurrentIntegral(-currentAcc);
            if (trackCurrent <= 0)
            {
                currentIntegral = 0;
                trackCurrent    = 0;
            }
        }
    }

    goCurrent = trackCurrent;
}

/*
 * SetCurrentSetPoint
 * Sets the target current [mA] for MODE_COMMAND_CURRENT, clamped to the rated
 * current. Mirrors the Ctrl-Step-Driver Controller::SetCurrentSetPoint().
 */
void SetCurrentSetPoint(int32_t _cur)
{
    if      (_cur >  RATED_CURRENT_MA) goalCurrent =  RATED_CURRENT_MA;
    else if (_cur < -RATED_CURRENT_MA) goalCurrent = -RATED_CURRENT_MA;
    else                               goalCurrent = _cur;
}

/*
 * SetVelocitySetPoint
 * Sets the target velocity [subdivisions/s] for MODE_COMMAND_VELOCITY, clamped
 * to the working ratedVelocity. Mirrors Controller::SetVelocitySetPoint().
 */
void SetVelocitySetPoint(int32_t _vel)
{
    if      (_vel >  ratedVelocity) goalVelocity =  ratedVelocity;
    else if (_vel < -ratedVelocity) goalVelocity = -ratedVelocity;
    else                            goalVelocity = _vel;
}

/*
 * SetPositionSetPoint
 * Sets the target position [subdivisions] for MODE_COMMAND_POSITION, referenced
 * to the encoder home offset. Mirrors Controller::SetPositionSetPoint().
 */
void SetPositionSetPoint(int32_t _pos)
{
    goalPosition = _pos + encoderHomeOffset;
}

/*
 * SetPositionSetPointWithTime
 * Sets a target position to be reached in approximately _time seconds by
 * computing the cruise velocity of a trapezoidal profile with the configured
 * acceleration (velocityAcc). If the move cannot complete within _time at the
 * velocity limit, it clamps to velocityLimit and returns false; otherwise it
 * derives the required cruise velocity and returns true.
 * Direct port of Controller::SetPositionSetPointWithTime().
 */
bool SetPositionSetPointWithTime(int32_t _pos, float _time)
{
    int32_t deltaPos = abs(_pos - realPosition + encoderHomeOffset);

    /* Max distance reachable within _time under a full accel/decel triangle. */
    float pMax = (float)velocityAcc * _time * _time / 4.0f;

    if ((float)deltaPos > pMax)
    {
        /* Not reachable in time: run at the velocity limit. */
        ratedVelocity = velocityLimit;
        SetPositionSetPoint(_pos);
        return false;
    }
    else
    {
        /* Solve trapezoidal profile for the cruise velocity that hits the
         * target in exactly _time seconds. */
        float vMax = _time * (float)velocityAcc;
        vMax -= (float)velocityAcc *
                sqrtf(_time * _time - 4.0f * (float)deltaPos / (float)velocityAcc);
        vMax /= 2.0f;

        ratedVelocity = (int32_t)vMax;
        SetPositionSetPoint(_pos);
        return true;
    }
}

/*
 * CalcPidToOutput  (velocity PID – MODE_PWM_VELOCITY)
 * Computes PID output from the error between softVelocity and estVelocity,
 * then calls CalcCurrentToOutput to drive the FOC vector.
 */
static void CalcPidToOutput(int32_t _speed)
{
    pid.vErrorLast = pid.vError;
    pid.vError     = _speed - estVelocity;
    if (pid.vError >  (1024 * 1024)) pid.vError =  (1024 * 1024);
    if (pid.vError < -(1024 * 1024)) pid.vError = -(1024 * 1024);

    pid.outputKp = pid.kp * pid.vError;

    pid.integralRound    += pid.ki * pid.vError;
    pid.integralRemainder = pid.integralRound >> 10;
    pid.integralRound    -= (pid.integralRemainder << 10);
    pid.outputKi         += pid.integralRemainder;
    if (pid.outputKi >  (RATED_CURRENT_MA << 10)) pid.outputKi =  (RATED_CURRENT_MA << 10);
    if (pid.outputKi < -(RATED_CURRENT_MA << 10)) pid.outputKi = -(RATED_CURRENT_MA << 10);

    pid.outputKd = pid.kd * (pid.vError - pid.vErrorLast);

    pid.output = (pid.outputKp + pid.outputKi + pid.outputKd) >> 10;
    if (pid.output >  RATED_CURRENT_MA) pid.output =  RATED_CURRENT_MA;
    if (pid.output < -RATED_CURRENT_MA) pid.output = -RATED_CURRENT_MA;

    CalcCurrentToOutput(pid.output);
}

/*
 * CalcDceToOutput  (position DCE controller – MODE_COMMAND_POSITION)
 * Dynamic Compensation Equation: combines a position error term (kp/ki) with
 * a velocity error term (kv/kd) to produce the FOC current command. Mirrors
 * the Ctrl-Step-Driver reference CalcDceToOutput().
 */
static void CalcDceToOutput(int32_t _location, int32_t _speed)
{
    dce.pError = _location - estPosition;
    if (dce.pError >  3200) dce.pError =  3200;   /* limit pError to 1/16 rev (51200/16) */
    if (dce.pError < -3200) dce.pError = -3200;

    dce.vError = (_speed - estVelocity) >> 7;
    if (dce.vError >  4000) dce.vError =  4000;
    if (dce.vError < -4000) dce.vError = -4000;

    dce.outputKp = dce.kp * dce.pError;

    dce.integralRound    += (dce.ki * dce.pError + dce.kv * dce.vError);
    dce.integralRemainder = dce.integralRound >> 7;
    dce.integralRound    -= (dce.integralRemainder << 7);
    dce.outputKi         += dce.integralRemainder;
    if (dce.outputKi >  (RATED_CURRENT_MA << 10)) dce.outputKi =  (RATED_CURRENT_MA << 10);
    if (dce.outputKi < -(RATED_CURRENT_MA << 10)) dce.outputKi = -(RATED_CURRENT_MA << 10);

    dce.outputKd = dce.kd * dce.vError;

    dce.output = (dce.outputKp + dce.outputKi + dce.outputKd) >> 10;
    if (dce.output >  RATED_CURRENT_MA) dce.output =  RATED_CURRENT_MA;
    if (dce.output < -RATED_CURRENT_MA) dce.output = -RATED_CURRENT_MA;

    CalcCurrentToOutput(dce.output);
}

/*
 * Position-tracker sub-step accumulators (preserve fractional increments at 20 kHz).
 */
static void PosTracker_CalcPositionIntegral(int32_t _value)
{
    posPositionIntegral += _value;
    posTrackPosition    += posPositionIntegral / CONTROL_FREQUENCY;
    posPositionIntegral  = posPositionIntegral % CONTROL_FREQUENCY;
}

static void PosTracker_CalcVelocityIntegral(int32_t _value)
{
    posVelocityIntegral += _value;
    posTrackVelocity    += posVelocityIntegral / CONTROL_FREQUENCY;
    posVelocityIntegral  = posVelocityIntegral % CONTROL_FREQUENCY;
}

/*
 * PosTracker_SetVelocityAcc
 * Sets the trapezoidal accel/decel step and the pre-computed factor used to
 * estimate the braking distance ( v^2 * 0.5/decel ).
 */
static void PosTracker_SetVelocityAcc(int32_t _value)
{
    posVelocityUpAcc        = _value;
    posVelocityDownAcc      = _value;
    posQuickVelocityDownAcc = 0.5f / (float)posVelocityDownAcc;
}

/*
 * PosTracker_NewTask
 * Seeds the tracker from the current estimated position/velocity so the
 * trajectory starts smoothly (called when entering position mode).
 */
static void PosTracker_NewTask(int32_t _realLocation, int32_t _realSpeed)
{
    posVelocityIntegral = 0;
    posTrackVelocity    = _realSpeed;
    posPositionIntegral = 0;
    posTrackPosition    = _realLocation;
}

/*
 * PosTracker_CalcSoftGoal  (trapezoidal position planner – MODE_COMMAND_POSITION)
 * Generates a smooth position/velocity profile toward _goalPosition, ramping
 * velocity up to ratedVelocity and braking in time to stop on target.
 * Direct port of the Ctrl-Step-Driver PositionTracker::CalcSoftGoal().
 */
static void PosTracker_CalcSoftGoal(int32_t _goalPosition)
{
    int32_t deltaPosition = _goalPosition - posTrackPosition;

    if (deltaPosition == 0)
    {
        if ((posTrackVelocity >= -posSpeedLockingBrake) &&
            (posTrackVelocity <=  posSpeedLockingBrake))
        {
            posVelocityIntegral = 0;
            posTrackVelocity    = 0;
            posPositionIntegral = 0;
        }
        else if (posTrackVelocity > 0)
        {
            PosTracker_CalcVelocityIntegral(-posVelocityDownAcc);
            if (posTrackVelocity <= 0)
            {
                posVelocityIntegral = 0;
                posTrackVelocity    = 0;
            }
        }
        else if (posTrackVelocity < 0)
        {
            PosTracker_CalcVelocityIntegral(posVelocityDownAcc);
            if (posTrackVelocity >= 0)
            {
                posVelocityIntegral = 0;
                posTrackVelocity    = 0;
            }
        }
    }
    else
    {
        if (posTrackVelocity == 0)
        {
            if (deltaPosition > 0)
                PosTracker_CalcVelocityIntegral(posVelocityUpAcc);
            else
                PosTracker_CalcVelocityIntegral(-posVelocityUpAcc);
        }
        else if ((deltaPosition > 0) && (posTrackVelocity > 0))
        {
            if (posTrackVelocity <= ratedVelocity)
            {
                int32_t need_down_location = (int32_t)((float)posTrackVelocity *
                                                       (float)posTrackVelocity *
                                                       posQuickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (posTrackVelocity < ratedVelocity)
                    {
                        PosTracker_CalcVelocityIntegral(posVelocityUpAcc);
                        if (posTrackVelocity >= ratedVelocity)
                        {
                            posVelocityIntegral = 0;
                            posTrackVelocity    = ratedVelocity;
                        }
                    }
                    else if (posTrackVelocity > ratedVelocity)
                    {
                        PosTracker_CalcVelocityIntegral(-posVelocityDownAcc);
                    }
                }
                else
                {
                    PosTracker_CalcVelocityIntegral(-posVelocityDownAcc);
                    if (posTrackVelocity <= 0)
                    {
                        posVelocityIntegral = 0;
                        posTrackVelocity    = 0;
                    }
                }
            }
            else
            {
                PosTracker_CalcVelocityIntegral(-posVelocityDownAcc);
                if (posTrackVelocity <= 0)
                {
                    posVelocityIntegral = 0;
                    posTrackVelocity    = 0;
                }
            }
        }
        else if ((deltaPosition < 0) && (posTrackVelocity < 0))
        {
            if (posTrackVelocity >= -ratedVelocity)
            {
                int32_t need_down_location = (int32_t)((float)posTrackVelocity *
                                                       (float)posTrackVelocity *
                                                       posQuickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (posTrackVelocity > -ratedVelocity)
                    {
                        PosTracker_CalcVelocityIntegral(-posVelocityUpAcc);
                        if (posTrackVelocity <= -ratedVelocity)
                        {
                            posVelocityIntegral = 0;
                            posTrackVelocity    = -ratedVelocity;
                        }
                    }
                    else if (posTrackVelocity < -ratedVelocity)
                    {
                        PosTracker_CalcVelocityIntegral(posVelocityDownAcc);
                    }
                }
                else
                {
                    PosTracker_CalcVelocityIntegral(posVelocityDownAcc);
                    if (posTrackVelocity >= 0)
                    {
                        posVelocityIntegral = 0;
                        posTrackVelocity    = 0;
                    }
                }
            }
            else
            {
                PosTracker_CalcVelocityIntegral(posVelocityDownAcc);
                if (posTrackVelocity >= 0)
                {
                    posVelocityIntegral = 0;
                    posTrackVelocity    = 0;
                }
            }
        }
        else if ((deltaPosition < 0) && (posTrackVelocity > 0))
        {
            PosTracker_CalcVelocityIntegral(-posVelocityDownAcc);
            if (posTrackVelocity <= 0)
            {
                posVelocityIntegral = 0;
                posTrackVelocity    = 0;
            }
        }
        else if ((deltaPosition > 0) && (posTrackVelocity < 0))
        {
            PosTracker_CalcVelocityIntegral(posVelocityDownAcc);
            if (posTrackVelocity >= 0)
            {
                posVelocityIntegral = 0;
                posTrackVelocity    = 0;
            }
        }
    }

    PosTracker_CalcPositionIntegral(posTrackVelocity);

    posGoLocation = posTrackPosition;
    posGoVelocity = posTrackVelocity;
}

/*
 * HAL callback: 20 kHz timer interrupt – main FOC control tick.
 * Implements the MODE_PWM_VELOCITY path from the Ctrl-Step-Driver reference:
 *   1. Read encoder
 *   2. First-call position seed
 *   3. Wrap-around lap position update
 *   4. Multi-turn (naive) position accumulation
 *   5. Integer IIR velocity estimation
 *   6. Lead-angle compensation -> estimated position
 *   7. Velocity PID -> FOC current vector
 *   8. Soft-goal (trapezoidal ramp) update for next tick
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        /* Encoder calibration takes over the control loop when active */
        if (Calib_IsRunning())
        {
            Calib_Tick20kHz();
            return;
        }

//    	SetFocCurrentVector(a, b);

        /* 1. Read encoder */
        angleData = GetAngle();
        angleData_Calibrated = Calib_GetCalibratedAngleLUT(angleData); /* rectified position via calib table */

        /* 2. First-call initialisation – seed position tracking */
        static bool isFirstCalled = true;
        if (isFirstCalled)
        {
            isFirstCalled       = false;
            realLapPosition     = (int32_t)angleData_Calibrated;
            realLapPositionLast = (int32_t)angleData_Calibrated;
            realPosition        = (int32_t)angleData_Calibrated;
            realPositionLast    = (int32_t)angleData_Calibrated;
            return;
        }

        /* 3. Update lap position with wrap-around handling */
        realLapPositionLast = realLapPosition;
        realLapPosition     = (int32_t)angleData_Calibrated;

        int32_t deltaLapPosition = realLapPosition - realLapPositionLast;
        if (deltaLapPosition >  (MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1))
            deltaLapPosition -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        else if (deltaLapPosition < -(MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1))
            deltaLapPosition += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        /* 4. Accumulate multi-turn (naive) position */
        realPositionLast = realPosition;
        realPosition    += deltaLapPosition;

        /*
         * 5. Estimate velocity – integer IIR low-pass filter
         *    accumulator += deltaPos * Fs  +  estVelocity * 31
         *    estVelocity  = accumulator >> 5  (divide by 32)
         *    Remainder kept in accumulator to preserve sub-unit precision.
         */
        estVelocityIntegral += (realPosition - realPositionLast) * CONTROL_FREQUENCY
                               + ((estVelocity << 5) - estVelocity);
        estVelocity          = estVelocityIntegral >> 5;
        estVelocityIntegral -= (estVelocity << 5);

        /* 6. Lead-angle compensation -> estimated rotor position */
        estLeadPosition = CompensateAdvancedAngle(estVelocity);
        estPosition     = realPosition + estLeadPosition;

        /* 7. Run the active mode's controller -> FOC current vector */
        switch (runningMode)
        {
            case MODE_COMMAND_VELOCITY:
                CalcPidToOutput(softVelocity);
                break;
            case MODE_COMMAND_POSITION:
                CalcDceToOutput(softPosition, softVelocity);
                break;
            case MODE_COMMAND_CURRENT:
                CalcCurrentToOutput(softCurrent);
                break;
            default:
                break;
        }

        /* 8. Mode-change handling: seed the tracker on the first tick of a new mode */
        if (runningMode != requestMode)
        {
            runningMode  = requestMode;
            softNewCurve = true;
        }
        if (softNewCurve)
        {
            softNewCurve = false;
            switch (runningMode)
            {
                case MODE_COMMAND_VELOCITY:
                    velocityIntegral = 0;
                    trackVelocity    = estVelocity;
                    break;
                case MODE_COMMAND_POSITION:
                    PosTracker_NewTask(estPosition, estVelocity);
                    break;
                case MODE_COMMAND_CURRENT:
                    CurrentTracker_NewTask(focCurrent);
                    break;
                default:
                    break;
            }
        }

        /* 9. Advance the soft goal (trajectory generator) for the next tick */
        switch (runningMode)
        {
            case MODE_COMMAND_VELOCITY:
            {
                int32_t clampedVel = goalVelocity;
                if (clampedVel >  ratedVelocity) clampedVel =  ratedVelocity; // Remove clamping later, claper alreary clamp in SetVelocitySetPoint
                if (clampedVel < -ratedVelocity) clampedVel = -ratedVelocity;
                CalcSoftGoal(clampedVel);
                softVelocity = goVelocity;
                break;
            }
            case MODE_COMMAND_POSITION:
                PosTracker_CalcSoftGoal(goalPosition);
                softPosition = posGoLocation;
                softVelocity = posGoVelocity;
                break;
            case MODE_COMMAND_CURRENT:
            {
                int32_t clampedCur = goalCurrent;
                if (clampedCur >  RATED_CURRENT_MA) clampedCur =  RATED_CURRENT_MA; // Remove clamping later, claper alreary clamp in SetCurrentSetPoint
                if (clampedCur < -RATED_CURRENT_MA) clampedCur = -RATED_CURRENT_MA;
                CurrentTracker_CalcSoftGoal(clampedCur);
                softCurrent = goCurrent;
                break;
            }
            default:
                break;
        }

        /******************************** Update State ********************************/
        if (runningMode == MODE_COMMAND_POSITION)
		{
			if ((softPosition == goalPosition)
				&& (softVelocity == 0))
				StepperState = STATE_FINISH;
			else
				StepperState = STATE_RUNNING;
		} else if (runningMode == MODE_COMMAND_VELOCITY)
		{
			if (softVelocity == goalVelocity)
				StepperState = STATE_FINISH;
			else
				StepperState = STATE_RUNNING;
		} else if (runningMode == MODE_COMMAND_CURRENT)
		{
			if (softCurrent == goalCurrent)
				StepperState = STATE_FINISH;
			else
				StepperState = STATE_RUNNING;
		} else
		{
			StepperState = STATE_FINISH;
		}
    }
}


void CAN_SetVelocitySetPoint(uint8_t _val)
{
	uint8_t canBuf[8];
    uint8_t mode = 0x04;
	TxHeader.StdId = 1 << 7 | mode;

	canBuf[0] = _val;

    CAN_Send(&TxHeader, canBuf);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim3);

  /*---------- Load BoardConfig from flash ----------*/
  if (!BoardConfig_Load())
  {
    /* No valid config in flash – write defaults */
    BoardConfig_SetDefaults();
  }

  /* Apply boardConfig to runtime variables */
  velocityAcc   = boardConfig.velocityAcc;
  velocityLimit = boardConfig.velocityLimit;
  ratedVelocity = velocityLimit;
  currentAcc    = 10 * RATED_CURRENT_MA;

  /* Start encoder calibration. calibTablePtr will point to flash
   * once calibState == CALIB_DONE (checked in the main loop).
   * If valid calibration data exists in flash, use it directly
   * instead of re-running the calibration sequence. */
  if (Calib_LoadFromFlash())
  {
    /* Calibration table loaded from flash – skip motor calibration */
  }
  else
  {
    /* No valid flash data – run full calibration sequence */
    Calib_Start();
  }
  goalVelocity = 410000;
  goalPosition = 5000;
  requestMode = MODE_COMMAND_POSITION;

  /* Apply PID gains from boardConfig */
  pid.kp = boardConfig.pid_kp;
  pid.ki = boardConfig.pid_ki;
  pid.kd = boardConfig.pid_kd;

  /* Apply DCE gains from boardConfig */
  dce.kp = boardConfig.dce_kp;
  dce.kv = boardConfig.dce_kv;
  dce.ki = boardConfig.dce_ki;
  dce.kd = boardConfig.dce_kd;

  /* Position tracker: accel/decel step and locking-brake threshold */
  PosTracker_SetVelocityAcc(velocityAcc);
  CurrentTracker_SetCurrentAcc(ratedCurrentAcc);
  posSpeedLockingBrake = velocityAcc / 1000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Run calibration calculation (no-op after CALIB_DONE) */
    Calib_TickMainLoop();

    /* Handle config write-back requests */
    if (boardConfig.configStatus == CONFIG_COMMIT)
    {
      boardConfig.configStatus = CONFIG_OK;
      BoardConfig_Save();
    }
    else if (boardConfig.configStatus == CONFIG_RESTORE)
    {
      if (BoardConfig_Load() == false) {BoardConfig_SetDefaults();}
      HAL_NVIC_SystemReset();
    }
    else if (boardConfig.configStatus == CONFIG_DEFAULT)
    {
      BoardConfig_SetDefaults();
      HAL_NVIC_SystemReset();
    }

    /* Normal motion control starts once calibration is complete */
    
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
