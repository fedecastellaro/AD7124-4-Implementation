/*
 * ad7124_regs.h
 *
 *  Created on: 1 de oct. de 2018
 *      Author: root
 */

#ifndef AD7124_REGS_H_
#define AD7124_REGS_H_


#include "ad7124.h"

/*! Device register info */

typedef struct ad7124_st_reg {
	uint32_t addr;
	uint32_t value;
	uint32_t size;
	uint32_t rw;
}ad7124_st_reg;

/*! AD7124 registers list*/
enum ad7124_registers {
	AD7124_Status = 0x00,
	AD7124_ADC_Control,
	AD7124_Data,
	AD7124_IOCon1,
	AD7124_IOCon2,
	AD7124_ID,
	AD7124_Error,
	AD7124_Error_En,
	AD7124_Mclk_Count,
	AD7124_Channel_0,
	AD7124_Channel_1,
	AD7124_Channel_2,
	AD7124_Channel_3,
	AD7124_Channel_4,
	AD7124_Channel_5,
	AD7124_Channel_6,
	AD7124_Channel_7,
	AD7124_Channel_8,
	AD7124_Channel_9,
	AD7124_Channel_10,
	AD7124_Channel_11,
	AD7124_Channel_12,
	AD7124_Channel_13,
	AD7124_Channel_14,
	AD7124_Channel_15,
	AD7124_Config_0,
	AD7124_Config_1,
	AD7124_Config_2,
	AD7124_Config_3,
	AD7124_Config_4,
	AD7124_Config_5,
	AD7124_Config_6,
	AD7124_Config_7,
	AD7124_Filter_0,
	AD7124_Filter_1,
	AD7124_Filter_2,
	AD7124_Filter_3,
	AD7124_Filter_4,
	AD7124_Filter_5,
	AD7124_Filter_6,
	AD7124_Filter_7,
	AD7124_Offset_0,
	AD7124_Offset_1,
	AD7124_Offset_2,
	AD7124_Offset_3,
	AD7124_Offset_4,
	AD7124_Offset_5,
	AD7124_Offset_6,
	AD7124_Offset_7,
	AD7124_Gain_0,
	AD7124_Gain_1,
	AD7124_Gain_2,
	AD7124_Gain_3,
	AD7124_Gain_4,
	AD7124_Gain_5,
	AD7124_Gain_6,
	AD7124_Gain_7,
	AD7124_REG_NO
};


/*! Array holding the info for the ad7124 registers - address, initial value,
    size and access type. */

//2 ES READ, 1 READ/WRITE

/*! Array holding the info for the ad7124 registers - address, initial value,
size and access type. */
extern ad7124_st_reg ad7124_regs_init[AD7124_REG_NO];


#endif /* AD7124_REGS_H_ */
