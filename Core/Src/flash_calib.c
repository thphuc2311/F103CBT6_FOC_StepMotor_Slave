/*
 * flash_calib.c
 *
 * Flash storage for encoder calibration table on STM32F103CB.
 * Uses HAL flash programming interface (half-word writes).
 */

#include "flash_calib.h"
#include "encoder_calib.h"
#include "board_config.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Private state                                                             */
/* ------------------------------------------------------------------------- */
static uint32_t writeAddress = 0U;

/* ------------------------------------------------------------------------- */
/* Internal helpers                                                          */
/* ------------------------------------------------------------------------- */

/* Erase a single flash page at the given address.
 * HAL_FLASHEx_Erase already waits for completion and clears PER bit,
 * so no extra FLASH_WaitForLastOperation / CLEAR_BIT needed. */
static void FlashCalib_ErasePage(uint32_t addr)
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0U;

    eraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress  = addr;
    eraseInit.NbPages      = 1U;

    HAL_FLASHEx_Erase(&eraseInit, &pageError);
}

/* Erase all pages covering [addr, addr + size). */
static void FlashCalib_EraseRange(uint32_t addr, uint32_t size)
{
    uint32_t numPages = (size + CALIB_FLASH_PAGE_SIZE - 1U) / CALIB_FLASH_PAGE_SIZE;
    for (uint32_t i = 0U; i < numPages; i++)
        FlashCalib_ErasePage(addr + i * CALIB_FLASH_PAGE_SIZE);
}

/* ------------------------------------------------------------------------- */
/* Public API – calibration table (APP_CALI)                                */
/* ------------------------------------------------------------------------- */

void FlashCalib_ClearTable(void)
{
    HAL_FLASH_Unlock();
    FlashCalib_EraseRange(APP_CALI_ADDR, APP_CALI_SIZE);
    HAL_FLASH_Lock();
}

void FlashCalib_BeginWrite(void)
{
    HAL_FLASH_Unlock();
    FlashCalib_EraseRange(APP_CALI_ADDR, APP_CALI_SIZE);
    writeAddress = APP_CALI_ADDR;
}

void FlashCalib_Write16(uint16_t data)
{
    if (writeAddress < APP_CALI_ADDR)
        return;
    if (writeAddress >= (APP_CALI_ADDR + APP_CALI_SIZE))
        return;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          writeAddress, (uint64_t)data) == HAL_OK)
    {
        writeAddress += 2U;
    }
}

void FlashCalib_EndWrite(void)
{
    HAL_FLASH_Lock();
}

bool FlashCalib_IsValid(void)
{
    /* Check 1: boardConfig.calibStatus == true (in APP_DATA) */
    BoardConfig_t cfg;
    FlashUserData_Read(&cfg, sizeof(cfg));
    if (!cfg.calibStatus)
        return false;

    /* Check 2: scan LUT for 0xFFFF (erased entries) */
    return FlashCalib_IsTableWritten();
}

bool FlashCalib_IsTableWritten(void)
{
    volatile uint16_t *flashPtr = (volatile uint16_t *)APP_CALI_ADDR;
    for (uint32_t i = 0U; i < ENCODER_RESOLUTION; i++)
    {
        if (flashPtr[i] == 0xFFFFU)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* Public API – runtime config (APP_DATA)                                   */
/* ------------------------------------------------------------------------- */

void FlashUserData_Erase(void)
{
    HAL_FLASH_Unlock();
    FlashCalib_EraseRange(APP_DATA_ADDR, APP_DATA_SIZE);
    HAL_FLASH_Lock();
}

void FlashUserData_Read(void *data, uint32_t size)
{
    if (size > APP_DATA_SIZE)
        return;

    memcpy(data, (const void *)APP_DATA_ADDR, size);
}

void FlashUserData_Write(const void *data, uint32_t size)
{
    if (size > APP_DATA_SIZE)
        return;

    /* 1. Read entire page into RAM buffer */
    static uint8_t pageBuffer[CALIB_FLASH_PAGE_SIZE];
    memcpy(pageBuffer, (const void *)APP_DATA_ADDR, CALIB_FLASH_PAGE_SIZE);

    /* 2. Modify the desired region in RAM buffer (offset 0) */
    memcpy(pageBuffer, data, size);

    /* 3. Disable interrupts – flash erase/program stalls the CPU bus.
    *    A 20 kHz TIM3 ISR firing during flash operations can cause
    *    a HardFault or leave the flash controller in an inconsistent
    *    state.  Critical section covers erase + program. */
    __disable_irq();

    /* 4. Erase flash page */
    HAL_FLASH_Unlock();
    FlashCalib_EraseRange(APP_DATA_ADDR, APP_DATA_SIZE);

    /* 5. Program entire buffer back to flash (half-word writes) */
    for (uint32_t i = 0U; i < CALIB_FLASH_PAGE_SIZE; i += 2U)
    {
        uint16_t halfWord = (uint16_t)pageBuffer[i] | ((uint16_t)pageBuffer[i + 1U] << 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          APP_DATA_ADDR + i, (uint64_t)halfWord);
    }

    HAL_FLASH_Lock();

    /* 6. Re-enable interrupts */
    __enable_irq();
}
