/*
 * ad7124.c
 *
 *  Created on: 13 de oct. de 2018
 *      Author: root
 */

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ad7124.h"

extern double calibracion[21];

/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */


#define		VALUE_200mmHg		1.453052
#define		VALUE_150mmHg		1.113188
#define		VALUE_100mmHg		0.776806
#define		VALUE_90mmHg		0.712020
#define		VALUE_80mmHg		0.643585
#define 	AD7124_INIT_OK		1
#define 	AD7124_INIT_ERR		-1

#define		ERROR_FACTOR		(1.25 * 1 * 0.018 * 2.998)
#define		kPa2mmHg			7.500615758456564
#define		SENSITIVITY
#define		ZERO				0.1414

							/*FUNCIONES PROPIAS*/

/*************************************************************************************/

/**
 * @brief Sets the ADC Control register
 * @param mode Control the mode of operation for ADC
 * @param power_mode Power mode
 * @param ref_en Internal reference voltage enable. When this bit is set,
 * the internal reference is enabled and available at the REFOUT pin.
 * When this bit is cleared, the internal reference is disabled.
 * @param clk_sel select the clock source for the ADC
 * @return 0 for success or negative error code
 */

uint32_t Ad7124Chip_setAdcControl (OperatingMode mode,
                           	   	   PowerMode power_mode,
							   	   bool ref_en, ClkSel clk_sel) {

	uint32_t aux = 0;
	uint8_t buffer[8];
	uint8_t i = 0;


	aux = AD7124_ADC_CTRL_REG_MODE (mode) |
             AD7124_ADC_CTRL_REG_POWER_MODE (power_mode) |
             AD7124_ADC_CTRL_REG_CLK_SEL (clk_sel) |
             (ref_en ? AD7124_ADC_CTRL_REG_REF_EN : 0) |
             AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;


	InitBuffer (buffer,0xaa);
    COMMAND = WR_REGISTER(AD7124_ADC_Control);
    BYTE_1  = ( (aux & 0xFF00) >> 8);
    BYTE_2  = ( (aux & 0x00FF));
    i =	3;

	tx_n_rx(buffer, i);

	return aux;

}
/***********************************************************************************/
/**
    * @brief Sets a setup
    * @param cfg Setup select.
    * @param ref Reference source select bits.
    * @param pga Gain select bits.
    * @param bipolar Polarity select bit. When this bit is set, bipolar
    * operation is selected. When this bit is cleared, unipolar operation is
    * selected.
    * @param burnout These bits select the magnitude of the sensor burnout detect current source.
    * @return 0 for success or negative error code
    */

int32_t Ad7124Chip_setConfig (uint8_t cfg, RefSel ref, PgaSel pga,
                       bool bipolar, BurnoutCurrent burnout) {

	uint32_t aux = 0;
	uint8_t buffer[8];
	uint8_t i = 0;

	if (cfg < 8) {
    cfg += AD7124_Config_0;

    aux =    AD7124_CFG_REG_REF_SEL (ref) |
                  AD7124_CFG_REG_PGA (pga) |
                  (bipolar ? AD7124_CFG_REG_BIPOLAR : 0) |
                  AD7124_CFG_REG_BURNOUT (burnout) |
                  AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM |
                  AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;

	InitBuffer (buffer,0xaa);
    COMMAND = WR_REGISTER(cfg);
    BYTE_1  = ( (aux & 0xFF00) >> 8);
    BYTE_2  = ( (aux & 0x00FF));
    i =	3;

	tx_n_rx(buffer, i);

    return  (aux);
  }
  return -1;
}
/***********************************************************************************/

/**
 * @brief Setup channel
 * @param ch channel number 0 to 15
 * @param cfg Setup select. These bits identify which of the eight setups are used to configure the ADC for this channel.
 * @param ainp Positive analog input AINP input select.
 * @param ainm Negative analog input AINM input select.
 * @param enable Channel enable bit. Setting this bit enables the device channel for the conversion sequence.
 * @return 0 for success or negative error code
 */

uint32_t Ad7124Chip_setChannel (uint8_t ch, uint8_t cfg, InputSel ainp,
                        InputSel ainm, bool enable) {

	uint32_t aux = 0;
	uint8_t buffer[8];
	uint8_t i = 0;

	if ( (ch < 16) && (cfg < 8)) {

    ch += AD7124_Channel_0;

    aux = AD7124_CH_MAP_REG_SETUP (cfg) |
               AD7124_CH_MAP_REG_AINP (ainp) |
               AD7124_CH_MAP_REG_AINM (ainm) |
               (enable ? AD7124_CH_MAP_REG_CH_ENABLE : 0);

	InitBuffer (buffer,0xaa);
    COMMAND = WR_REGISTER(ch);
    BYTE_1  = ( (aux & 0xFF00) >> 8);
    BYTE_2  = ( (aux & 0x00FF));
    i =	3;

	tx_n_rx(buffer, i);
  }
  return aux;
}
/***********************************************************************************/

double Ad7124Chip_toVoltage (long value, int gain, double vref, bool bipolar)
{
  double voltage = (double) value;

  if (bipolar)
    voltage = (voltage / (double) 0x800000) - 1;

  else
    voltage = voltage / (double) 0x1000000;


  voltage = voltage * vref / (double) gain;
  return voltage;


}
/***********************************************************************************/
/**
 * @brief Sampling a channel
 * The channel is enabled in single mode, then the conversion is started
 * and the value of the sample is returned
 * @param ch channel number
 * @return sample or negative error code
 */

long Ad7124Chip_read (uint8_t ch)
{
	Ad7124Chip_startSingleConversion (ch);

	return Ad7124Chip_getData();
}
/***********************************************************************************/
/**
  * @brief Start conversion in single mode
  * @param ch channel number
  * @return 0 for success or negative error code
  */

void Ad7124Chip_startSingleConversion (uint8_t ch)
{
	Ad7124Chip_setAdcControl   (SingleConvMode, FullPower, TRUE, InternalClk);
	Ad7124Chip_setChannel      (CHANNEL_0, SETUP_0, AIN0Input, AIN1Input, TRUE); //revisar este último false!!

  return ;
}
/***********************************************************************************/
/**
 * @brief Enable/Disable channel
 * @param ch channel number 0 to 15
 * @param enable true for enabled
 * @return 0 for success or negative error code
 */

void Ad7124Chip_enableChannel (uint8_t ch, bool enable) {

  if (ch < 16) {

		uint32_t aux = 0;
		uint8_t buffer[8];
		uint8_t i = 0;

    ch += AD7124_Channel_0;

    if (enable) {

    	aux |= AD7124_CH_MAP_REG_CH_ENABLE;

    	InitBuffer (buffer,0xaa);
        COMMAND = WR_REGISTER(ch);
        BYTE_1  = ( (aux & 0xFF00) >> 8);
        BYTE_2  = ( (aux & 0x00FF));
        i =	3;

    	tx_n_rx(buffer, i);

    }
    else {

    	aux &= ~AD7124_CH_MAP_REG_CH_ENABLE;

    	InitBuffer (buffer,0xaa);
        COMMAND = WR_REGISTER(ch);
        BYTE_1  = ( (aux & 0xFF00) >> 8);
        BYTE_2  = ( (aux & 0x00FF));
        i =	3;

    	tx_n_rx(buffer, i);

    }

  }
  return;
}

/***********************************************************************************/
/**
 * @brief Control the mode of operation for ADC
 * @param mode mode of operation
 * @return 0 for success or negative error code
 */

void Ad7124Chip_setMode (OperatingMode mode)
{
	uint32_t aux = 0;
	uint8_t buffer[8];
	uint8_t i = 0;

	aux &= ~AD7124_ADC_CTRL_REG_MODE (0x0F); // clear mode
	aux |=  AD7124_ADC_CTRL_REG_MODE (mode);

	InitBuffer (buffer,0xaa);

    COMMAND = WR_REGISTER(AD7124_ADC_Control);
    BYTE_1  = ( (aux & 0xFF00) >> 8);
    BYTE_2  = ( (aux & 0x00FF));
    i =	3;

	tx_n_rx(buffer, i);

	return;
}

/*********************************************************************************/

void Ad7124_begin (void)
{
	uint8_t buffer[8];
	uint8_t reg_nr;
	/*  Reset the device interface.*/
	//ad7124_reset();


	/* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */

	for(reg_nr = AD7124_Status; (reg_nr < AD7124_REG_NO); reg_nr++)
	{
		if (ad7124_regs_init[reg_nr].rw == AD7124_RW)
		{
			InitBuffer (buffer,0xaa);
		    COMMAND = WR_REGISTER(ad7124_regs_init[reg_nr].addr);

			if (ad7124_regs_init[reg_nr].size == 1 )
				BYTE_1  = ad7124_regs_init[reg_nr].value;


			else if (ad7124_regs_init[reg_nr].size == 2 )
			{
				BYTE_1  = (ad7124_regs_init[reg_nr].value & 0xFF00) >> 8;
				BYTE_2  = (ad7124_regs_init[reg_nr].value & 0x00FF);
			}

			else if (ad7124_regs_init[reg_nr].size == 3 )
			{
				BYTE_1  = (ad7124_regs_init[reg_nr].value & 0xFF0000) >> 16;
				BYTE_2  = (ad7124_regs_init[reg_nr].value & 0x00FF00) >> 8;
				BYTE_3  = (ad7124_regs_init[reg_nr].value & 0x0000FF);
			}

			tx_n_rx(buffer, ad7124_regs_init[reg_nr].size + 1);
		}
	}

	Ad7124Chip_setAdcControl (StandbyMode, LowPower, FALSE ,InternalClk);
	return;
}

/***********************************************************************************/
/**
 * @brief Returns the last sample
 * @return sample or negative error code
 */

long Ad7124Chip_getData(void)
{
	uint8_t buffer[8];
	uint8_t i = 0;
	bool ready = false;


	  do {
	    /* Read the value of the Status Register */
			InitBuffer (buffer,0xAA);
			COMMAND = RD_REGISTER(AD7124_Status);
			i =	2;
			tx_n_rx(buffer, i);

	    /* Check the RDY bit in the Status Register */
	    ready = (buffer[0] &
	             AD7124_STATUS_REG_RDY) == 0;

	  	  }	 while (!ready);


		//InitBuffer (buffer,0xAA);
		COMMAND = RD_REGISTER(AD7124_Data);
		i =	4;

		tx_n_rx(buffer, i);

	return chnd_2_nmbr(buffer,0);
}

/*********************************************************************************************/


void Ad7124Chip_Offset_Gain_Calibration(uint32_t *offset,uint32_t *gain)
{
	uint8_t buffer[8];
	uint8_t i = 0;
	uint32_t aux = 0;
	bool ready = false;

	aux = Ad7124Chip_setAdcControl (InternalOffsetCalibrationMode, LowPower, TRUE, InternalClk);

			  do {
				/* Read the value of the Status Register */
					InitBuffer (buffer,0xAA);
					COMMAND = RD_REGISTER(AD7124_ADC_Control);
					i =	3;
					tx_n_rx(buffer, i);

				/* Check the RDY bit in the Status Register */
				ready = (buffer[0] &
						AD7124_ADC_CTRL_REG_DOUT_RDY_DEL) == 0;

				  }	 while (!ready);

				COMMAND = RD_REGISTER(AD7124_Offset_0);
				i =	4;
				tx_n_rx(buffer, i);

				offset = (uint32_t)chnd_2_nmbr(buffer,0);

	aux = Ad7124Chip_setAdcControl (StandbyMode, LowPower, TRUE, InternalClk);
	aux = Ad7124Chip_setAdcControl (InternalGainCalibrationMode, LowPower, TRUE, InternalClk);

			  do {
				/* Read the value of the Status Register */
					InitBuffer (buffer,0xAA);
					COMMAND = RD_REGISTER(AD7124_ADC_Control);
					i =	3;
					tx_n_rx(buffer, i);

				/* Check the RDY bit in the Status Register */
				ready = (buffer[0] &
						AD7124_ADC_CTRL_REG_DOUT_RDY_DEL) == 0;

				  }	 while (!ready);

				COMMAND = RD_REGISTER(AD7124_Gain_0);
				i =	4;
				tx_n_rx(buffer, i);

				gain = (uint32_t)chnd_2_nmbr(buffer,0);
}


/******************************************************************************************/

uint8_t Ad7124Chip_ReadID(void)
{
	uint8_t buffer[8];
	uint8_t i = 0;

	COMMAND = RD_REGISTER(AD7124_ID);
	i =	2;
	tx_n_rx(buffer, i);

	return buffer[0];
}

/***********************************************************************************/

void SPI_Init(void){
	SSP_ConfigFormat ssp_format;

	Board_SSP_Init(LPC_SSP);
	Chip_SSP_Init(LPC_SSP);
	Chip_SSP_SetMaster(LPC_SSP, 1);
	Chip_SSP_SetBitRate(LPC_SSP,1000000); // Default es de 100000, max testeado 40000000
	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_BITS_8;
	ssp_format.clockMode = SSP_CLOCK_MODE3;
	Chip_SSP_SetFormat(LPC_SSP, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);
	Chip_SSP_Enable(LPC_SSP);

	NVIC_EnableIRQ(SSP1_IRQn);

	}

/***********************************************************************************/


void GPIOs_Init(void)
{
	/**************************   SSP  ***************************/
	Chip_IOCON_PinMux(LPC_IOCON, CS_SPP , IOCON_MODE_PULLDOWN, IOCON_FUNC0);
	Chip_IOCON_PinMux(LPC_IOCON, SYNC_ADC , IOCON_MODE_PULLDOWN, IOCON_FUNC0);

	Chip_GPIO_SetPinDIROutput(LPCP0, SYNC_ADC);
	Chip_GPIO_SetPinDIROutput(LPCP0, CS_SPP);

	Chip_GPIO_WritePortBit(LPCP0, SYNC_ADC , TRUE);
	Chip_GPIO_WritePortBit(LPCP0, CS_SPP   , TRUE);

	/*************************************************************/
}

/***********************************************************************************/

void InitBuffer(unsigned char * buff,uint8_t fill)
{
	uint16_t i;

	for(i=0;i<8;i++)
			buff[i]=fill;
}

/***********************************************************************************/

void tx_n_rx(uint8_t *data, uint8_t length )
{
    Chip_SSP_DATA_SETUP_T xferConfig;
    uint8_t rxbuffer[8];
    uint8_t i;

	InitBuffer (rxbuffer, 0xaa);

	xferConfig.rx_cnt  = xferConfig.tx_cnt = 0;
	xferConfig.tx_data = data;
	xferConfig.rx_data = rxbuffer;
	xferConfig.length  = length;

	SSP_CS_LOW;

	Chip_SSP_RWFrames_Blocking(LPC_SSP, &xferConfig);

	delay(10);

	SSP_CS_HIGH;

	for (i = 0; i < length-1 ; i++)
		data[i] = rxbuffer[i+1];

	return;
}

/***********************************************************************************/

long chnd_2_nmbr (uint8_t *rxbuffer, uint8_t indice)
{
	int32_t aux = 0;

	aux |= ((rxbuffer[indice] & 0xFF) << 16);
	aux |= ((rxbuffer[indice+1] & 0xFF) << 8);
	aux |= ((rxbuffer[indice+2] & 0xFF));

	return (long)aux;
}

/***********************************************************************************/

void delay (uint32_t time)
{
	//time = 1; 1us
	//time = 10; 2us
	//time = 100; 10us
	uint32_t i;
	for (i = 0; i <= time ; i++ );
	return;
}

double Ad7124Chip_toPressure(double voltage)
{
	double pressure_mmHg = voltage- ZERO ;

		pressure_mmHg /= 2.998;
	//	pressure_mmHg -= 0.04;
		pressure_mmHg /= 0.0185555;
		pressure_mmHg *= kPa2mmHg;

	return pressure_mmHg;


}

/*
 *
 * Presion_cal = lim_min + ( (lim_max - lim_min) / (v_lim_max - v_lim_min) ) * (voltaje - v_lim_min)
 *
 */

double toPressure ( double voltage )
{
//0 = 0.14342
//200 = 1.54540

	//calibrado por tramo

	double p = ((200-20)*(voltage-0.25681)/(1.54540-0.25681))+20;

	if ( p <= 205 )
	{
		if (p <= 160)
		{
			if (p <= 120)
			{
				if (p <= 80)
				{
					if (p <= 40)
						p -= 0.5; //escala 0 -40 //OK
					else
						p -= 1; //escala 40 -80//OK
				}
				else
					p -= 1.5;	//escala 80 -120//OK
			}
			else
				p -= 1.1;	//escala 120 -160//OK
		}
		else
			p -= 0.2; //escala 160-205//OK
	}
	else
		return -1;

	if(p<0)
		return 0;

	return p;

//
//		uint8_t i = 0;
//
//		if ( voltage < 0.14299 )
//			return 0;
//
//	while (i < 3)
//	{
//		if( voltage < calibracion[i] )
//			return (80*i + ( (80) / (calibracion[i+1] - calibracion[i]) ) * (voltage - calibracion[i]))+20;
//
//		i++;
//	}
//	return -1;

}

int AD7124_Init (void)
{
	uint8_t ad7124_CHIP_ID = Ad7124Chip_ReadID();

	//uint32_t aux = 0;
	uint8_t buffer[8];
	uint8_t i = 0;


	if ( ad7124_CHIP_ID == 4 || ad7124_CHIP_ID == 6 )
	{
	Ad7124Chip_setAdcControl   (StandbyMode, FullPower, TRUE, InternalClk);
	Ad7124Chip_setConfig 	   (SETUP_0, RefAVdd, Pga1, FALSE, BurnoutOff);
	Ad7124Chip_setChannel      (CHANNEL_0, SETUP_0, AIN0Input, AIN1Input, TRUE); //revisar este último false!!

	Ad7124Chip_setAdcControl   (ContinuousMode, FullPower, TRUE, InternalClk);

	InitBuffer (buffer,0xaa);
	COMMAND = WR_REGISTER(AD7124_Filter_0);
	BYTE_1= 0x00;
	BYTE_2= 0x00;
	BYTE_3= 0x01;
	i =	4;

	tx_n_rx(buffer, i);

	return AD7124_INIT_OK;
	}

	else
		return AD7124_INIT_ERR;
}


