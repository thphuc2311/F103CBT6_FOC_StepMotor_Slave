#include <stdbool.h>
#include <stdlib.h>
#include "Driver_tb67h450.h"
#include "sin_map.h"

static uint16_t SinMapIndex_PhaseA = 0, SinMapIndex_PhaseB = 0;
static int16_t SinMapValues_PhaseA = 0, SinMapValues_PhaseB = 0;
extern int16_t SinMapValues_PhaseA_debug1, SinMapValues_PhaseB_debug1;
extern int16_t SinMapValues_PhaseA_debug2, SinMapValues_PhaseB_debug2;

static void DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, _voltageA_3300mVIn12bits >> 2); // Scale to 10bit // PA1 ->A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, _voltageB_3300mVIn12bits >> 2); // Scale to 10bit // PA2 ->B
}

static void SetInputA(bool _statusA_plus, bool _statusA_minus)
{
	_statusA_plus 	? (Output_Ap_GPIO_Port->BSRR = Output_Ap_Pin) : (Output_Ap_GPIO_Port->BSRR = (uint32_t)Output_Ap_Pin << 16U); //PB12
	_statusA_minus 	? (Output_Am_GPIO_Port->BSRR = Output_Am_Pin) : (Output_Am_GPIO_Port->BSRR = (uint32_t)Output_Am_Pin << 16U); //PB13
}

static void SetInputB(bool _statusB_plus, bool _statusB_minus)
{
	_statusB_plus 	? (Output_Bp_GPIO_Port->BSRR = Output_Bp_Pin) : (Output_Bp_GPIO_Port->BSRR = (uint32_t)Output_Bp_Pin << 16U); //PB14
	_statusB_minus 	? (Output_Bm_GPIO_Port->BSRR = Output_Bm_Pin) : (Output_Bm_GPIO_Port->BSRR = (uint32_t)Output_Bm_Pin << 16U); //PB15
}

void SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA)
{
	SinMapIndex_PhaseB = _directionInCount & (0x000003FF); // Limit in 10bit
	SinMapIndex_PhaseA = (SinMapIndex_PhaseB + (256)) & (0x000003FF); // Make phase shift 90 degree

	SinMapValues_PhaseA = sin_pi_m2[SinMapIndex_PhaseA];
	SinMapValues_PhaseB = sin_pi_m2[SinMapIndex_PhaseB];

	uint32_t Amplitude = abs(_current_mA); // Bien do
	Amplitude = (Amplitude * 4095) / 3300; // Map 3300 mA to 12 bit resolution
	Amplitude = Amplitude & (0x00000FFF);  // make sure not out of boundary 12 bit

	uint16_t PhaseA_dacValue12Bits = (uint32_t)(Amplitude * abs(SinMapValues_PhaseA)) >> sin_pi_m2_dpiybit;
	uint16_t PhaseB_dacValue12Bits = (uint32_t)(Amplitude * abs(SinMapValues_PhaseB)) >> sin_pi_m2_dpiybit;

	SinMapValues_PhaseA_debug1 = PhaseA_dacValue12Bits;
	SinMapValues_PhaseB_debug1 = PhaseB_dacValue12Bits;
	SinMapValues_PhaseA_debug2 = (uint32_t)((Amplitude + 500) * abs(SinMapValues_PhaseA)) >> sin_pi_m2_dpiybit;
	SinMapValues_PhaseB_debug2 = (uint32_t)((Amplitude + 500) * abs(SinMapValues_PhaseB)) >> sin_pi_m2_dpiybit;;

	DacOutputVoltage(PhaseA_dacValue12Bits,PhaseB_dacValue12Bits);

	if (SinMapValues_PhaseA > 0)
		SetInputA(true, false);
	else if (SinMapValues_PhaseA < 0)
		SetInputA(false, true);
	else
		SetInputA(true, true);

	if (SinMapValues_PhaseB > 0)
		SetInputB(true, false);
	else if (SinMapValues_PhaseB < 0)
		SetInputB(false, true);
	else
		SetInputB(true, true);
}

void Driver_Sleep()
{
	DacOutputVoltage(0,0);
    SetInputA(false, false);
    SetInputB(false, false);
}
