/*
 * NTC_10K_LUT.h
 *
 *  Created on: Oct 18, 2026
 *      Author: Admin
 */

#ifndef INC_NTC_10K_LUT_H_
#define INC_NTC_10K_LUT_H_

#include <stdio.h>

#define R_FIXED 3300.0f // Dien tro co dinh 3.3k
#define ADC_MAX 4095.0f // ADC 12 bit

typedef struct
{
	float temperature;
	float NTC_Resistor;
}NTC_Temp_Res_t;
// define sai roi, tach ra thanh 2 bang nhiet do va dien tro. -> hoac van dung, pending de kiem tra.
const NTC_Temp_Res_t ntc_tem_res_table[] =
{
		{25.0f		,10000.0f	},
		{30.0f		,8023.8f	},
		{35.0f		,6484.3f	},
		{40.0f		,5276.0f	},
		{45.0f		,4320.7f	},
		{50.0f		,3560.4f	},
		{55.0f		,2951.2f	},
		{60.0f		,2460.0f	},
		{65.0f		,2061.7f	},
		{70.0f		,1736.8f	},
		{75.0f		,1470.3f	},
		{80.0f		,1250.6f	},
		{85.0f		,1068.5f	},
		{90.0f		,916.9f		},
		{95.0f		,790.1f		},
		{100.0f		,683.6f		},
		{105.0f		,593.6f		},
		{110.0f		,517.5f		},
		{115.0f		,452.7f		},
};

#define NTC_TABLE_SIZE (sizeof(ntc_tem_res_table))/sizeof(NTC_Temp_Res_t)
#endif /* INC_NTC_10K_LUT_H_ */
