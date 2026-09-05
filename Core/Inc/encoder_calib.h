#ifndef ENCODER_CALIB_H
#define ENCODER_CALIB_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Encoder & motor parameters
 * ------------------------------------------------------------------------- */
#define ENCODER_RESOLUTION      16384U   /* MT6701: 14-bit = 16384 counts/rev  */
#define CALIB_HARD_STEPS        200U     /* 1.8° stepper: 200 full steps/rev   */
#define CALIB_SOFT_DIVIDE_NUM   256U     /* micro-step subdivision factor       */
#define CALIB_SUBDIVIDE_STEPS   (CALIB_HARD_STEPS * CALIB_SOFT_DIVIDE_NUM) /* 51200 */
#define CALIB_SAMPLE_PER_STEP   16U      /* encoder samples averaged per step   */
#define CALIB_AUTO_SPEED        2U       /* subdivisions/tick during prepare    */
#define CALIB_FINE_SPEED        1U       /* subdivisions/tick during measure    */
#define CALIB_CURRENT_MA        1500     /* drive current during calibration    */

/*
 * Tick prescaler: the 20 kHz ISR calls Calib_Tick20kHz() every interrupt, but
 * the calibration state machine only advances once every CALIB_TICK_DIVIDER
 * calls.  This slows the motor motion so the rotor has time to settle at each
 * micro-step (less vibration, more accurate samples) without changing the
 * timer configuration.
 *   Effective logical rate = 20 kHz / CALIB_TICK_DIVIDER
 *   e.g. 20 -> 1 kHz logical tick.  Larger = slower & gentler but longer calib.
 */
#define CALIB_TICK_DIVIDER      20U

/* -------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */
typedef enum
{
    CALIB_ERR_NONE = 0,
    CALIB_ERR_AVG_DIR,          /* direction / zero-crossing inconsistency     */
    CALIB_ERR_AVG_CONTINUITY,   /* delta between adjacent steps out of range   */
    CALIB_ERR_PHASE_STEP,       /* wrap-around step count != 1                 */
    CALIB_ERR_QUANTITY,         /* total table entries != ENCODER_RESOLUTION   */
} CalibError_t;

typedef enum
{
    CALIB_IDLE = 0,
    CALIB_FWD_PREPARE,       /* one full CW revolution to lock rotor           */
    CALIB_FWD_MEASURE,       /* CW scan: sample encoder at every hard step     */
    CALIB_BWD_RETURN,        /* overshoot past start to eliminate backlash     */
    CALIB_BWD_GAP_DISMISS,   /* reverse back to measurement start position     */
    CALIB_BWD_MEASURE,       /* CCW scan: sample encoder at every hard step    */
    CALIB_CALCULATING,       /* ISR idles motor; main loop builds table        */
    CALIB_DONE,              /* table valid in encoderCalibTable[]             */
} CalibState_t;

/* -------------------------------------------------------------------------
 * Output: rectification lookup table (RAM)
 *   encoderCalibTable[raw_angle_14bit] = motor_subdivide_position
 * Total size: 16384 * 2 bytes = 32 KB
 * ------------------------------------------------------------------------- */
extern uint16_t          encoderCalibTable[ENCODER_RESOLUTION];
extern volatile CalibError_t calibError;
extern volatile CalibState_t calibState;

/* -------------------------------------------------------------------------
 * Diagnostics (valid after a calibration attempt – inspect in debugger)
 *   Ideal per-hard-step encoder delta = ENCODER_RESOLUTION / CALIB_HARD_STEPS
 *                                      = 16384 / 200 ~= 82 counts.
 *   If calibDbgFailDelta is far from ~82, the step-angle / encoder scale
 *   (CALIB_HARD_STEPS) or the wiring is wrong:
 *     ~82  -> correct (1.8 deg, 200-step motor)
 *     ~41  -> motor is 0.9 deg (400-step): set CALIB_HARD_STEPS = 400
 *     ~0   -> motor not moving: raise CALIB_CURRENT_MA / check wiring
 * ------------------------------------------------------------------------- */
extern volatile int32_t calibDbgMinDelta;   /* smallest |delta| seen           */
extern volatile int32_t calibDbgMaxDelta;   /* largest  |delta| seen           */
extern volatile int32_t calibDbgFailIndex;  /* hard-step index that failed (-1) */
extern volatile int32_t calibDbgFailDelta;  /* signed delta at failing step     */
extern volatile int32_t calibDbgFailCount;  /* number of out-of-band steps       */
extern volatile int32_t calibDbgSumDelta;   /* sum of signed deltas over 1 rev   */

/* Raw averaged forward-pass samples (encoder counts at each hard step).     */
/* Watch this array in the debugger to see the actual angle progression.     */
extern uint16_t calibDbgForward[CALIB_HARD_STEPS + 1U];

/* -------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */
/* Call once from main() after peripherals are ready to begin calibration. */
void Calib_Start(void);

/* Call from the 20 kHz timer ISR. Drives the motor and collects samples.   */
void Calib_Tick20kHz(void);

/* Call continuously from the main loop. Runs the table-build calculation   */
/* when sampling is complete; transitions to CALIB_DONE when finished.      */
void Calib_TickMainLoop(void);

/* Returns true while calibration is in progress (ISR must call Tick20kHz). */
bool Calib_IsRunning(void);

#endif /* ENCODER_CALIB_H */
