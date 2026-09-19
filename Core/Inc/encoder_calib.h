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
    CALIB_START = 0,
    CALIB_FWD_PREPARE,       /* one full CW revolution to lock rotor           */
    CALIB_FWD_MEASURE,       /* CW scan: sample encoder at every hard step     */
    CALIB_BWD_RETURN,        /* overshoot past start to eliminate backlash     */
    CALIB_BWD_GAP_DISMISS,   /* reverse back to measurement start position     */
    CALIB_BWD_MEASURE,       /* CCW scan: sample encoder at every hard step    */
    CALIB_CALCULATING,       /* ISR idles motor; main loop builds table        */
    CALIB_DONE,              /* table valid in encoderCalibTable[]             */
} CalibState_t;

/* -------------------------------------------------------------------------
 * Output: rectification lookup table (in flash, accessed via pointer)
 *   calibTablePtr[raw_angle_14bit] = motor_subdivide_position
 * Total size: 16384 * 2 bytes = 32 KB
 *
 * The table lives in flash (APP_CALI).  Because F103CB has only 20 KB RAM,
 * the 32 KB table cannot be mirrored in RAM.  Instead a pointer
 * (calibTablePtr) points directly into flash memory-mapped space.
 *
 * During calibration, BuildCalibTable() writes to flash via FlashCalib_*.
 * After calibration (or on boot from flash), calibTablePtr is set to
 * APP_CALI_ADDR and reads are direct flash reads.
 * ------------------------------------------------------------------------- */
extern volatile uint16_t *calibTablePtr;
extern volatile CalibError_t calibError;
extern volatile CalibState_t calibState;

/* True after a valid calibration table has been loaded from flash or built. */
extern bool calibTableValid;

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

/* Read a single calibrated position from the table.
 * Reads directly from flash via calibTablePtr. */
uint16_t Calib_GetCalibratedAngleLUT(uint16_t rawAngle);

/* Load calibration table from flash.
 * Sets calibTablePtr to point at APP_CALI in flash.
 * Returns true if valid data was found, false otherwise.
 * Call once at startup before using Calib_GetCalibratedAngleLUT(). */
bool Calib_LoadFromFlash(void);

/* Save the calibration table to flash.
 * Call after a successful calibration (calibState == CALIB_DONE,
 * calibError == CALIB_ERR_NONE). */
void Calib_SaveToFlash(void);

/* Returns true if a valid calibration table is available (flash). */
bool Calib_IsTableValid(void);

#endif /* ENCODER_CALIB_H */
