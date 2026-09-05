/*
 * flash_calib.h
 *
 * Flash storage for encoder calibration table on STM32F103CB.
 *
 * Flash layout (128 KB total, 1 KB pages) — matches reference project:
 *   0x08000000 - 0x08017BFF   APP_FIRMWARE   (95 KB, pages 0-94)
 *   0x08017C00 - 0x0801FBFF   APP_CALI       (32 KB, pages 95-126)
 *   0x0801FC00 - 0x0801FFFF   APP_DATA       ( 1 KB, page 127)
 *
 * APP_CALI  stores the 16384 × 2-byte calibration lookup table.
 * APP_DATA  stores runtime configuration (boardConfig) including a
 *           calibration-valid flag so the table is only used when valid.
 */

#ifndef FLASH_CALIB_H
#define FLASH_CALIB_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* Flash address definitions                                                 */
/* ------------------------------------------------------------------------- */

/* APP_FIRMWARE */
#define APP_FIRMWARE_ADDR            0x08000000U
#define APP_FIRMWARE_SIZE            0x00017C00U   /* 95 KB */

/* APP_CALI – calibration lookup table */
#define APP_CALI_ADDR                0x08017C00U
#define APP_CALI_SIZE                0x00008000U   /* 32 KB */

/* APP_DATA – runtime config (boardConfig)
 *
 * Layout within APP_DATA (1 KB page):
 *   offset 0x00:  BoardConfig_t  (calibStatus, PID gains, limits, ...)
 */
#define APP_DATA_ADDR                0x0801FC00U
#define APP_DATA_SIZE                0x00000400U   /* 1 KB */

/* STM32F1 flash page size (1 KB for medium-density MCUs) */
#define CALIB_FLASH_PAGE_SIZE        0x400U

/* ------------------------------------------------------------------------- */
/* API – calibration table (APP_CALI)                                       */
/* ------------------------------------------------------------------------- */

/* Erase the calibration table region. */
void FlashCalib_ClearTable(void);

/* Begin a flash write session for the calibration table
 * (unlock, erase APP_CALI pages, set write pointer). */
void FlashCalib_BeginWrite(void);

/* Write a single uint16_t to flash and advance the internal write pointer. */
void FlashCalib_Write16(uint16_t data);

/* End a flash write session (lock flash). */
void FlashCalib_EndWrite(void);

/* Check whether valid calibration data exists.
 * Uses two checks (both must pass):
 *   1. boardConfig.calibStatus == true (in APP_DATA)
 *   2. No 0xFFFF entries in APP_CALI (table not empty/erased) */
bool FlashCalib_IsValid(void);

/* Scan the entire LUT in APP_CALI for 0xFFFF entries.
 * Returns true if the table is fully written (no 0xFFFF found),
 * false if any entry is still erased. */
bool FlashCalib_IsTableWritten(void);

/* ------------------------------------------------------------------------- */
/* API – runtime config (APP_DATA)                                          */
/* ------------------------------------------------------------------------- */

/* Erase the APP_DATA page. */
void FlashUserData_Erase(void);

/* Read BoardConfig_t from APP_DATA into a buffer. */
void FlashUserData_Read(void *data, uint32_t size);

/* Write data to APP_DATA (erases page first).
 * Reads existing content into RAM buffer, modifies, then erases + programs. */
void FlashUserData_Write(const void *data, uint32_t size);

#endif /* FLASH_CALIB_H */
