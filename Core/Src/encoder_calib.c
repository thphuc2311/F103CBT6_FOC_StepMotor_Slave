/*
 * encoder_calib.c
 *
 * Encoder calibration for MT6701 (14-bit) + 1.8° stepper with TB67H450 driver.
 *
 * Algorithm adapted from:
 *   Ctrl-Step-Driver-STM32F1-fw / Ctrl/Sensor/Encoder/encoder_calibrator_base.*
 *   (original: C++, flash storage)
 *
 * Changes vs. original:
 *   - Ported to plain C
 *   - Calibration table stored in RAM (uint16_t encoderCalibTable[16384])
 *     instead of internal flash
 *
 * How the table is used after calibration:
 *   uint16_t motorPos = encoderCalibTable[GetAngle()];
 *   // motorPos is in [0, 51199] – use it as the encoder-corrected position
 */

#include "encoder_calib.h"
#include "Encoder_mt6701.h"
#include "Driver_tb67h450.h"
#include <stdlib.h>   /* abs() */

/* ==========================================================================
 * Public state
 * ========================================================================== */
uint16_t          encoderCalibTable[ENCODER_RESOLUTION];
volatile CalibError_t calibError = CALIB_ERR_NONE;
volatile CalibState_t calibState  = CALIB_IDLE;

/* ==========================================================================
 * Private state  (only written from ISR during their respective states,
 *                 read from main loop only after CALIB_CALCULATING)
 * ========================================================================== */
static uint32_t goPosition  = 0;
static bool     goDirection = false;
static uint16_t sampleCount = 0;
static uint32_t resultNum   = 0;
static int32_t  rcdX        = 0;
static int32_t  rcdY        = 0;

/* Intermediate sampling buffers */
static uint16_t sampleDataRaw            [CALIB_SAMPLE_PER_STEP];
static uint16_t sampleDataAverageForward [CALIB_HARD_STEPS + 1U]; /* index 0..200 */
static uint16_t sampleDataAverageBackward[CALIB_HARD_STEPS + 1U]; /* index 0..200 */

/* ==========================================================================
 * Cycle-safe math helpers
 * ========================================================================== */

/* (a + b) % b  –  normalises a positive integer into [0, b-1] */
static uint32_t CycleMod(uint32_t a, uint32_t b)
{
    return (a + b) % b;
}

/* Signed subtraction that respects cyclic wrap-around */
static int32_t CycleSubtract(int32_t a, int32_t b, int32_t cyc)
{
    int32_t diff = a - b;
    if (diff >  (cyc >> 1)) diff -= cyc;
    if (diff < -(cyc >> 1)) diff += cyc;
    return diff;
}

/* Average of two cyclic values */
static int32_t CycleAverage(int32_t a, int32_t b, int32_t cyc)
{
    int32_t sub = a - b;
    int32_t avg = (a + b) >> 1;
    if (abs(sub) > (cyc >> 1))
    {
        if (avg >= (cyc >> 1)) avg -= (cyc >> 1);
        else                   avg += (cyc >> 1);
    }
    return avg;
}

/* Average of an array of cyclic uint16_t samples */
static int32_t CycleDataAverage(const uint16_t *data, uint16_t len, int32_t cyc)
{
    int32_t sum = (int32_t)data[0];
    for (uint16_t i = 1; i < len; i++)
    {
        int32_t sub  = (int32_t)data[i] - (int32_t)data[0];
        int32_t diff = (int32_t)data[i];
        if (sub >  (cyc >> 1)) diff = (int32_t)data[i] - cyc;
        if (sub < -(cyc >> 1)) diff = (int32_t)data[i] + cyc;
        sum += diff;
    }
    sum /= (int32_t)len;
    if (sum < 0)    sum += cyc;
    if (sum > cyc)  sum -= cyc;
    return sum;
}

/* ==========================================================================
 * Step 1 – data validation
 *
 * Merges forward and backward passes, checks monotonicity, and locates the
 * single encoder wrap-around step (rcdX, rcdY).
 * Sets calibError on failure.
 * ========================================================================== */
static void CalibrationDataCheck(void)
{
    uint32_t count;
    int32_t  subData;
    const int32_t calibSampleRes = (int32_t)ENCODER_RESOLUTION / (int32_t)CALIB_HARD_STEPS; /* ~81 */

    /* --- Average forward and backward passes -------------------------------- */
    for (count = 0U; count < CALIB_HARD_STEPS + 1U; count++)
    {
        sampleDataAverageForward[count] = (uint16_t)CycleAverage(
            (int32_t)sampleDataAverageForward[count],
            (int32_t)sampleDataAverageBackward[count],
            (int32_t)ENCODER_RESOLUTION);
    }

    /* --- Determine encoder direction ---------------------------------------- */
    subData = CycleSubtract((int32_t)sampleDataAverageForward[0],
                            (int32_t)sampleDataAverageForward[CALIB_HARD_STEPS - 1U],
                            (int32_t)ENCODER_RESOLUTION);
    if (subData == 0)
    {
        calibError = CALIB_ERR_AVG_DIR;
        return;
    }
    goDirection = (subData > 0);

    for (count = 1U; count < CALIB_HARD_STEPS; count++)
    {
        subData = CycleSubtract((int32_t)sampleDataAverageForward[count],
                                (int32_t)sampleDataAverageForward[count - 1U],
                                (int32_t)ENCODER_RESOLUTION);
        if (abs(subData) > (calibSampleRes * 3 / 2)) { calibError = CALIB_ERR_AVG_CONTINUITY; return; }
        if (abs(subData) < (calibSampleRes * 1 / 2)) { calibError = CALIB_ERR_AVG_CONTINUITY; return; }
        if (subData == 0)                             { calibError = CALIB_ERR_AVG_DIR;        return; }
        if ((subData > 0) && (!goDirection))          { calibError = CALIB_ERR_AVG_DIR;        return; }
        if ((subData < 0) && ( goDirection))          { calibError = CALIB_ERR_AVG_DIR;        return; }
    }

    /* --- Locate the single encoder wrap-around step (rcdX, rcdY) ------------ */
    uint32_t step_num = 0U;
    if (goDirection)
    {
        for (count = 0U; count < CALIB_HARD_STEPS; count++)
        {
            subData = (int32_t)sampleDataAverageForward[CycleMod(count + 1U, CALIB_HARD_STEPS)]
                    - (int32_t)sampleDataAverageForward[CycleMod(count,      CALIB_HARD_STEPS)];
            if (subData < 0)
            {
                step_num++;
                rcdX = (int32_t)count;
                rcdY = (int32_t)(ENCODER_RESOLUTION - 1U)
                     - (int32_t)sampleDataAverageForward[CycleMod((uint32_t)rcdX, CALIB_HARD_STEPS)];
            }
        }
    }
    else
    {
        for (count = 0U; count < CALIB_HARD_STEPS; count++)
        {
            subData = (int32_t)sampleDataAverageForward[CycleMod(count + 1U, CALIB_HARD_STEPS)]
                    - (int32_t)sampleDataAverageForward[CycleMod(count,      CALIB_HARD_STEPS)];
            if (subData > 0)
            {
                step_num++;
                rcdX = (int32_t)count;
                rcdY = (int32_t)(ENCODER_RESOLUTION - 1U)
                     - (int32_t)sampleDataAverageForward[CycleMod((uint32_t)(rcdX + 1), CALIB_HARD_STEPS)];
            }
        }
    }

    if (step_num != 1U)
    {
        calibError = CALIB_ERR_PHASE_STEP;
        return;
    }

    calibError = CALIB_ERR_NONE;
}

/* ==========================================================================
 * Step 2 – build lookup table in RAM
 *
 * Fills encoderCalibTable[raw_angle] = motor_subdivide_position.
 * resultNum must equal ENCODER_RESOLUTION when done.
 * ========================================================================== */
static void BuildCalibTable(void)
{
    int32_t  stepX, stepY, dataI32;
    uint16_t dataU16;
    resultNum = 0U;

    if (goDirection)
    {
        /* CW encoder direction */
        for (stepX = rcdX; stepX < rcdX + (int32_t)CALIB_HARD_STEPS + 1; stepX++)
        {
            dataI32 = CycleSubtract(
                (int32_t)sampleDataAverageForward[CycleMod((uint32_t)(stepX + 1), CALIB_HARD_STEPS)],
                (int32_t)sampleDataAverageForward[CycleMod((uint32_t) stepX,      CALIB_HARD_STEPS)],
                (int32_t)ENCODER_RESOLUTION);

            /* stepX == rcdX:              start at rcdY, end at dataI32  */
            /* stepX == rcdX+HARD_STEPS:   start at 0,    end at rcdY     */
            /* otherwise:                  start at 0,    end at dataI32  */
            int32_t startY = (stepX == rcdX)                              ? rcdY    : 0;
            int32_t endY   = (stepX == rcdX + (int32_t)CALIB_HARD_STEPS) ? rcdY    : dataI32;

            for (stepY = startY; stepY < endY; stepY++)
            {
                dataU16 = (uint16_t)CycleMod(
                    (uint32_t)((int32_t)CALIB_SOFT_DIVIDE_NUM *  stepX
                             + (int32_t)CALIB_SOFT_DIVIDE_NUM *  stepY / dataI32),
                    CALIB_SUBDIVIDE_STEPS);
                encoderCalibTable[resultNum++] = dataU16;
            }
        }
    }
    else
    {
        /* CCW encoder direction */
        for (stepX = rcdX + (int32_t)CALIB_HARD_STEPS; stepX > rcdX - 1; stepX--)
        {
            dataI32 = CycleSubtract(
                (int32_t)sampleDataAverageForward[CycleMod((uint32_t) stepX,      CALIB_HARD_STEPS)],
                (int32_t)sampleDataAverageForward[CycleMod((uint32_t)(stepX + 1), CALIB_HARD_STEPS)],
                (int32_t)ENCODER_RESOLUTION);

            /* stepX == rcdX+HARD_STEPS:  start at rcdY, end at dataI32  */
            /* stepX == rcdX:             start at 0,    end at rcdY     */
            /* otherwise:                 start at 0,    end at dataI32  */
            int32_t startY = (stepX == rcdX + (int32_t)CALIB_HARD_STEPS) ? rcdY    : 0;
            int32_t endY   = (stepX == rcdX)                              ? rcdY    : dataI32;

            for (stepY = startY; stepY < endY; stepY++)
            {
                dataU16 = (uint16_t)CycleMod(
                    (uint32_t)((int32_t)CALIB_SOFT_DIVIDE_NUM * (stepX + 1)
                             - (int32_t)CALIB_SOFT_DIVIDE_NUM *  stepY / dataI32),
                    CALIB_SUBDIVIDE_STEPS);
                encoderCalibTable[resultNum++] = dataU16;
            }
        }
    }

    if (resultNum != (uint32_t)ENCODER_RESOLUTION)
        calibError = CALIB_ERR_QUANTITY;
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

bool Calib_IsRunning(void)
{
    CalibState_t s = calibState;
    return (s != CALIB_IDLE) && (s != CALIB_DONE);
}

void Calib_Start(void)
{
    goPosition  = CALIB_SUBDIVIDE_STEPS;
    sampleCount = 0U;
    resultNum   = 0U;
    rcdX        = 0;
    rcdY        = 0;
    calibError  = CALIB_ERR_NONE;
    SetFocCurrentVector(goPosition, CALIB_CURRENT_MA);
    calibState  = CALIB_FWD_PREPARE;
}

/*
 * Calib_Tick20kHz – call from 20 kHz timer ISR while Calib_IsRunning().
 *
 * State machine mirrors encoder_calibrator_base::Tick20kHz().
 * All phases drive the motor via SetFocCurrentVector and collect raw encoder
 * samples.  CALIB_CALCULATING holds the motor at (0,0) while the main loop
 * runs BuildCalibTable.
 */
void Calib_Tick20kHz(void)
{
    /* Prescaler: advance the state machine only every CALIB_TICK_DIVIDER
     * interrupts so the motor has time to settle between micro-steps.
     * On the skipped ticks the motor simply holds its last energised vector. */
//    static uint16_t tickDivider = 0U;
//    if (++tickDivider < CALIB_TICK_DIVIDER)
//        return;
//    tickDivider = 0U;

    uint16_t rawAngle = GetAngle();

    switch (calibState)
    {
        /* ------------------------------------------------------------------
         * FWD_PREPARE: rotate one full CW revolution to seat the rotor.
         * goPosition: SUBDIVIDE_STEPS → 2×SUBDIVIDE_STEPS, then reset.
         * ------------------------------------------------------------------ */
        case CALIB_FWD_PREPARE:
            goPosition += CALIB_AUTO_SPEED;
            SetFocCurrentVector(goPosition, CALIB_CURRENT_MA);
            if (goPosition == 2U * CALIB_SUBDIVIDE_STEPS)
            {
                goPosition  = CALIB_SUBDIVIDE_STEPS;
                sampleCount = 0U;
                calibState  = CALIB_FWD_MEASURE;
            }
            break;

        /* ------------------------------------------------------------------
         * FWD_MEASURE: advance 1 sub-step per tick; at every hard-step
         * boundary (goPosition multiple of SOFT_DIVIDE_NUM) collect
         * SAMPLE_PER_STEP raw encoder readings and store their cyclic average.
         * ------------------------------------------------------------------ */
        case CALIB_FWD_MEASURE:
            if ((goPosition % CALIB_SOFT_DIVIDE_NUM) == 0U)
            {
                sampleDataRaw[sampleCount++] = rawAngle;
                if (sampleCount == CALIB_SAMPLE_PER_STEP)
                {
                    uint32_t idx = (goPosition - CALIB_SUBDIVIDE_STEPS) / CALIB_SOFT_DIVIDE_NUM;
                    sampleDataAverageForward[idx] = (uint16_t)CycleDataAverage(
                        sampleDataRaw, CALIB_SAMPLE_PER_STEP, (int32_t)ENCODER_RESOLUTION);
                    sampleCount = 0U;
                    goPosition += CALIB_FINE_SPEED;
                }
                /* else: hold position, collect more samples next tick */
            }
            else
            {
                goPosition += CALIB_FINE_SPEED;
            }
            SetFocCurrentVector(goPosition, CALIB_CURRENT_MA);
            if (goPosition > 2U * CALIB_SUBDIVIDE_STEPS)
                calibState = CALIB_BWD_RETURN;
            break;

        /* ------------------------------------------------------------------
         * BWD_RETURN: overshoot 20 hard steps past measurement start to
         * eliminate mechanical backlash before the backward pass.
         * ------------------------------------------------------------------ */
        case CALIB_BWD_RETURN:
            goPosition += CALIB_FINE_SPEED;
            SetFocCurrentVector(goPosition, CALIB_CURRENT_MA);
            if (goPosition == (2U * CALIB_SUBDIVIDE_STEPS + CALIB_SOFT_DIVIDE_NUM * 20U))
                calibState = CALIB_BWD_GAP_DISMISS;
            break;

        /* ------------------------------------------------------------------
         * BWD_GAP_DISMISS: return exactly to 2×SUBDIVIDE_STEPS.
         * ------------------------------------------------------------------ */
        case CALIB_BWD_GAP_DISMISS:
            goPosition -= CALIB_FINE_SPEED;
            SetFocCurrentVector(goPosition, CALIB_CURRENT_MA);
            if (goPosition == 2U * CALIB_SUBDIVIDE_STEPS)
            {
                sampleCount = 0U;
                calibState  = CALIB_BWD_MEASURE;
            }
            break;

        /* ------------------------------------------------------------------
         * BWD_MEASURE: mirror of FWD_MEASURE but stepping CCW.
         * Fills sampleDataAverageBackward[200..0].
         * ------------------------------------------------------------------ */
        case CALIB_BWD_MEASURE:
            if ((goPosition % CALIB_SOFT_DIVIDE_NUM) == 0U)
            {
                sampleDataRaw[sampleCount++] = rawAngle;
                if (sampleCount == CALIB_SAMPLE_PER_STEP)
                {
                    uint32_t idx = (goPosition - CALIB_SUBDIVIDE_STEPS) / CALIB_SOFT_DIVIDE_NUM;
                    sampleDataAverageBackward[idx] = (uint16_t)CycleDataAverage(
                        sampleDataRaw, CALIB_SAMPLE_PER_STEP, (int32_t)ENCODER_RESOLUTION);
                    sampleCount = 0U;
                    goPosition -= CALIB_FINE_SPEED;
                }
                /* else: hold position, collect more samples next tick */
            }
            else
            {
                goPosition -= CALIB_FINE_SPEED;
            }
            SetFocCurrentVector(goPosition, CALIB_CURRENT_MA);
            if (goPosition < CALIB_SUBDIVIDE_STEPS)
                calibState = CALIB_CALCULATING;
            break;

        /* ------------------------------------------------------------------
         * CALIB_CALCULATING: motor held at zero while main loop builds table.
         * ------------------------------------------------------------------ */
        case CALIB_CALCULATING:
            SetFocCurrentVector(0U, 0);
            break;

        default:
            break;
    }
}

/*
 * Calib_TickMainLoop – call from the main while-loop.
 *
 * Does nothing until the ISR has completed all sampling (CALIB_CALCULATING).
 * Then runs CalibrationDataCheck + BuildCalibTable (heavy computation, ~ms)
 * and transitions to CALIB_DONE.  After this Calib_IsRunning() returns false
 * and normal FOC control resumes.
 *
 * Check calibError afterwards:
 *   CALIB_ERR_NONE   → encoderCalibTable[] is valid
 *   anything else    → calibration failed; do not use the table
 */
void Calib_TickMainLoop(void)
{
    if (calibState != CALIB_CALCULATING)
        return;
    Driver_Sleep();
    CalibrationDataCheck();

    if (calibError == CALIB_ERR_NONE)
        BuildCalibTable();

    calibState = CALIB_DONE;
}
