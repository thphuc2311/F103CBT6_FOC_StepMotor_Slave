#include "Encoder_mt6701.h"

static const uint16_t TxData[2] = {0xFFFF, 0xFFFF};
static uint16_t RxData[2] = {0};
uint16_t GetAngle()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // Chip select
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)TxData, (uint8_t*)RxData, 2, 5); // 2 word 16-bit = 32 clock
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	// 32 clock: b31 = 1 bit dummy, khung 24-bit that nam o [30:7]
	//   angle[13:0]=[30:17] | Mg[3:0]=[16:13] | CRC[5:0]=[12:7]
	uint32_t raw = ((uint32_t)RxData[0] << 16) | RxData[1];
	return (uint16_t)((raw >> 17) & 0x3FFF);
}

/*
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // Chip select
	HAL_SPI_TransmitReceive(&hspi2, TxData, RxData, 3, 5);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	// MT6701 SSI: co 1 bit dummy dan dau => angle 14-bit that nam o [22:9]
	uint32_t raw = ((uint32_t)RxData[0] << 16) | ((uint32_t)RxData[1] << 8) | RxData[2];
	return (uint16_t)((raw >> 9) & 0x3FFF);
*/
