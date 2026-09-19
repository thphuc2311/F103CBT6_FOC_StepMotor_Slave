/*
 * board_config.h
 *
 * Board configuration stored in flash (APP_DATA, 1 KB page 127).
 * Adapted from Ctrl-Step-Driver-STM32F1-fw / UserApp/configurations.h.
 *
 * Changes vs. original:
 *   - Ported to plain C
 *   - Removed enableMotorOnBoot, enableStallProtect
 *   - Added PID gains (kp, ki, kd) for velocity loop
 *   - Added calibStatus field (replaces CALIB_VALID_SENTINEL mechanism)
 *
 * configStatus controls the write-back flow:
 *   CONFIG_RESTORE → write defaults to flash, then reboot
 *   CONFIG_OK      → normal operation (loaded from flash successfully)
 *   CONFIG_COMMIT  → user changed settings; write to flash, then set OK
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* Config status                                                             */
/* ------------------------------------------------------------------------- */
typedef enum
{
    CONFIG_RESTORE = 0,   /* restore to current in-flash config & reboot  */
    CONFIG_DEFAULT,       /* back to default config & reboot */
    CONFIG_OK,            /* normal operation         */
    CONFIG_COMMIT,        /* write to flash & set OK   */
} ConfigStatus_t;

/* ------------------------------------------------------------------------- */
/* Board configuration structure                                             */

/*                                                                         *
 * Stored at APP_DATA_ADDR in flash.                                       *
 * Total size must fit within APP_DATA (1 KB).                             *
 * ----------------------------------------------------------------------- */
typedef struct
{
    ConfigStatus_t configStatus;     /* write-back trigger flag             */
    uint32_t       canNodeId;        /* CAN node ID                         */
    int32_t        encoderHomeOffset;/* encoder zero-position offset        */
    uint32_t       defaultMode;      /* default control mode (Mode_t)       */
    int32_t        currentLimit;     /* rated current [mA]                  */
    int32_t        velocityLimit;    /* max velocity [subdivisions/s]       */
    int32_t        velocityAcc;      /* acceleration [subdivisions/s per tick] */
    int32_t        calibrationCurrent;/* calibration drive current [mA]     */
    int16_t        pid_kp;           /* velocity PID proportional gain      */
    int16_t        pid_ki;           /* velocity PID integral gain          */
    int16_t        pid_kd;           /* velocity PID derivative gain        */
    int32_t        dce_kp;           /* position DCE proportional gain      */
    int32_t        dce_kv;           /* position DCE velocity gain          */
    int32_t        dce_ki;           /* position DCE integral gain          */
    int32_t        dce_kd;           /* position DCE derivative gain        */
    bool           calibStatus;      /* true if calibration table is valid  */
} BoardConfig_t;

/* Global instance */
extern BoardConfig_t boardConfig;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

/* Load boardConfig from flash.  Returns true if valid data was found.
 * If not found, boardConfig is left at zeroed/default and the caller
 * should call BoardConfig_SetDefaults(). */
bool BoardConfig_Load(void);

/* Save the current boardConfig to flash (erase APP_DATA + program). */
void BoardConfig_Save(void);

/* Fill boardConfig with default values and save to flash. */
void BoardConfig_SetDefaults(void);

/* Mark config as needing commit (CONFIG_COMMIT).
 * The main loop will write it to flash and set CONFIG_OK. */
void BoardConfig_RequestCommit(void);

/* Mark config as needing restore (CONFIG_RESTORE).
 * The main loop will write defaults to flash and reboot. */
void BoardConfig_RequestRestore(void);

#endif /* BOARD_CONFIG_H */
