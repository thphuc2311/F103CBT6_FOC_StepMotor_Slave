/*
 * board_config.c
 *
 * Board configuration load/save to flash (APP_DATA).
 */

#include "board_config.h"
#include "flash_calib.h"
#include "can.h"
#include "main.h"

/* ------------------------------------------------------------------------- */
/* Global instance                                                            */
/* ------------------------------------------------------------------------- */
BoardConfig_t boardConfig;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

bool BoardConfig_Load(void)
{
    FlashUserData_Read(&boardConfig, sizeof(boardConfig));

    /* If configStatus is not a valid value, treat as not configured */
    if (boardConfig.configStatus != CONFIG_OK &&
        boardConfig.configStatus != CONFIG_COMMIT &&
        boardConfig.configStatus != CONFIG_RESTORE)
    {
        return false;
    }

    return true;
}

void BoardConfig_Save(void)
{
    FlashUserData_Write(&boardConfig, sizeof(boardConfig));
}

void BoardConfig_SetDefaults(void)
{
    boardConfig.configStatus       = CONFIG_OK;
    boardConfig.canNodeId          = ThisCAN_NodeId;
    boardConfig.encoderHomeOffset  = 0;
    boardConfig.defaultMode        = MODE_COMMAND_POSITION;    /* MODE_COMMAND_POSITION */
    boardConfig.currentLimit       = 1 * 1000;   /* mA */
    boardConfig.velocityLimit      = 8 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;      /* set by main() */
    boardConfig.velocityAcc        = 50 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;      /* set by main() */
    boardConfig.calibrationCurrent = 1500;   /* mA */
    boardConfig.pid_kp             = 8;
    boardConfig.pid_ki             = 50;
    boardConfig.pid_kd             = 100;
    boardConfig.dce_kp             = 1000;
    boardConfig.dce_kv             = 200;
    boardConfig.dce_ki             = 400;
    boardConfig.dce_kd             = 200;
    boardConfig.calibStatus        = false;
    BoardConfig_Save();
}

void BoardConfig_RequestCommit(void)
{
    boardConfig.configStatus = CONFIG_COMMIT;
}

void BoardConfig_RequestRestore(void)
{
    boardConfig.configStatus = CONFIG_RESTORE;
}
