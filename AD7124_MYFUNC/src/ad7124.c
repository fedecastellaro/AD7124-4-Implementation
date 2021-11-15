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

/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

/***************************************************************************//**
 * @brief Reads the value of the specified register without checking if the
 *        device is ready to accept user requests.
 *
 * @param dev   - The handler of the instance of the driver.
 * @param p_reg - Pointer to the register structure holding info about the
 *               register to be read. The read value is stored inside the
 *               register structure.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_no_check_read_register(struct ad7124_dev *dev,
				      struct ad7124_st_reg* p_reg)
{
	int32_t ret = 0;
	uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i = 0;
	uint8_t check8 = 0;
	uint8_t msg_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	if(!dev || !p_reg)
		return INVALID_VAL;

	/* Build the Command word */
	buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
		    AD7124_COMM_REG_RA(p_reg->addr);

	/* Read data from the device */
	ret = spi_write_and_read(
				 buffer,
				 ((dev->use_crc != AD7124_DISABLE_CRC) ? p_reg->size + 1
				  : p_reg->size) + 1);
	if(ret < 0)
		return ret;

	/* Check the CRC */
	if(dev->use_crc == AD7124_USE_CRC) {
		msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
			     AD7124_COMM_REG_RA(p_reg->addr);
		for(i = 1; i < p_reg->size + 2; ++i) {
			msg_buf[i] = buffer[i];
		}
		check8 = ad7124_compute_crc8(msg_buf, p_reg->size + 2);
	}

	if(check8 != 0) {
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/* Build the result */
	p_reg->value = 0;
	for(i = 1; i < p_reg->size + 1; i++) {
		p_reg->value <<= 8;
		p_reg->value += buffer[i];
	}

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register without checking if the
 *        device is ready to accept user requests.
 *
 * @param dev - The handler of the instance of the driver.
 * @param reg - Register structure holding info about the register to be written
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_no_check_write_register(struct ad7124_dev *dev,
				       struct ad7124_st_reg reg)
{
	int32_t ret = 0;
	int32_t reg_value = 0;
	uint8_t wr_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i = 0;
	uint8_t crc8 = 0;

	if(!dev)
		return INVALID_VAL;

	/* Build the Command word */
	wr_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR |
		    AD7124_COMM_REG_RA(reg.addr);

	/* Fill the write buffer */
	reg_value = reg.value;
	for(i = 0; i < reg.size; i++) {
		wr_buf[reg.size - i] = reg_value & 0xFF;
		reg_value >>= 8;
	}

	/* Compute the CRC */
	if(dev->use_crc != AD7124_DISABLE_CRC) {
		crc8 = ad7124_compute_crc8(wr_buf, reg.size + 1);
		wr_buf[reg.size + 1] = crc8;
	}

	/* Write data to the device */
	ret = spi_write_and_read(
				 wr_buf,
				 (dev->use_crc != AD7124_DISABLE_CRC) ? reg.size + 2
				 : reg.size + 1);

	return ret;
}

/***************************************************************************//**
 * @brief Reads the value of the specified register only when the device is ready
 *        to accept user requests. If the device ready flag is deactivated the
 *        read operation will be executed without checking the device state.
 *
 * @param dev   - The handler of the instance of the driver.
 * @param p_reg - Pointer to the register structure holding info about the
 *               register to be read. The read value is stored inside the
 *               register structure.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_read_register(struct ad7124_dev *dev,
			     struct ad7124_st_reg* p_reg)
{
	int32_t ret;

	if (p_reg->addr != AD7124_ERR_REG && dev->check_ready) {
		ret = ad7124_wait_for_spi_ready(dev,
						dev->spi_rdy_poll_cnt);
		if (ret < 0)
			return ret;
	}
	ret = ad7124_no_check_read_register(dev,
					    p_reg);

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register only when the device is
 *        ready to accept user requests. If the device ready flag is deactivated
 *        the write operation will be executed without checking the device state.
 *
 * @param dev - The handler of the instance of the driver.
 * @param reg - Register structure holding info about the register to be written
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_write_register(struct ad7124_dev *dev,
			      struct ad7124_st_reg p_reg)
{
	int32_t ret;

	if (dev->check_ready) {
		ret = ad7124_wait_for_spi_ready(dev,
						dev->spi_rdy_poll_cnt);
		if (ret < 0)
			return ret;
	}
	ret = ad7124_no_check_write_register(dev,
					     p_reg);

	return ret;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
void ad7124_reset(void)
{
	uint8_t wr_buf[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	tx_n_rx(wr_buf, 10);

	/* Wait for the reset to complete */
	ad7124_wait_to_power_on();

	return;
}

/***************************************************************************//**
 * @brief Waits until the device can accept read and write user actions.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_wait_for_spi_ready(struct ad7124_dev *dev,
				  uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!ready && --timeout) {
		/* Read the value of the Error Register */
		ret = ad7124_read_register(dev, &regs[AD7124_Error]);
		if(ret < 0)
			return ret;

		/* Check the SPI IGNORE Error bit in the Error Register */
		ready = (regs[AD7124_Error].value &
			 AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Waits until the device finishes the power-on reset operation.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
void ad7124_wait_to_power_on(void)
{
	uint8_t buffer [8];
	uint8_t i = 0;
	uint8_t powered_on = 0;

	while(!powered_on)
	{
		InitBuffer (buffer, 0xAA);
		COMMAND = RD_REGISTER(AD7124_Status);
		i = 2;
		tx_n_rx(buffer,i);

		/* Check the POR_FLAG bit in the Status Register */
		powered_on = (DATA_1 &
			      AD7124_STATUS_REG_POR_FLAG) == 0;
	}

	return;
}

/***************************************************************************//**
 * @brief Waits until a new conversion result is available.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns if no new data is available.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_wait_for_conv_ready(struct ad7124_dev *dev,
				   uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!ready && --timeout) {
		/* Read the value of the Status Register */
		ret = ad7124_read_register(dev, &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the RDY bit in the Status Register */
		ready = (regs[AD7124_Status].value &
			 AD7124_STATUS_REG_RDY) != 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Reads the conversion result from the device.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param p_data  - Pointer to store the read data.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_read_data(struct ad7124_dev *dev,
			 int32_t* p_data)
{
	struct ad7124_st_reg *regs;
	int32_t ret;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	/* Read the value of the Status Register */
	ret = ad7124_read_register(dev, &regs[AD7124_Data]);

	/* Get the read result */
	*p_data = regs[AD7124_Data].value;

	return ret;
}

/***************************************************************************//**
 * @brief Computes the CRC checksum for a data buffer.
 *
 * @param p_buf    - Data buffer
 * @param buf_size - Data buffer size in bytes
 *
 * @return Returns the computed CRC checksum.
*******************************************************************************/
uint8_t ad7124_compute_crc8(uint8_t * p_buf, uint8_t buf_size)
{
	uint8_t i = 0;
	uint8_t crc = 0;

	while(buf_size) {
		for(i = 0x80; i != 0; i >>= 1) {
			if(((crc & 0x80) != 0) != ((*p_buf & i) != 0)) { /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			} else {
				crc <<= 1;
			}
		}
		p_buf++;
		buf_size--;
	}
	return crc;
}

/***************************************************************************//**
 * @brief Updates the CRC settings.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return None.
*******************************************************************************/
void ad7124_update_crcsetting(struct ad7124_dev *dev)
{
	struct ad7124_st_reg *regs;

	if(!dev)
		return;

	regs = dev->regs;

	/* Get CRC State. */
	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_CRC_ERR_EN) {
		dev->use_crc = AD7124_USE_CRC;
	} else {
		dev->use_crc = AD7124_DISABLE_CRC;
	}
}

/***************************************************************************//**
 * @brief Updates the device SPI interface settings.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return None.
*******************************************************************************/
void ad7124_update_dev_spi_settings(struct ad7124_dev *dev)
{
	struct ad7124_st_reg *regs;

	if(!dev)
		return;

	regs = dev->regs;

	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN) {
		dev->check_ready = 1;
	} else {
		dev->check_ready = 0;
	}
}

/***************************************************************************//**
 * @brief Initializes the AD7124.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_setup(struct ad7124_dev **device,
		     struct ad7124_init_param init_param)
{
	int32_t ret;
	enum ad7124_registers reg_nr;
	struct ad7124_dev *dev;

	dev = (struct ad7124_dev *)malloc(sizeof(*dev));
	if (!dev)
		return INVALID_VAL;

	dev->regs = init_param.regs;
	dev->spi_rdy_poll_cnt = init_param.spi_rdy_poll_cnt;

	/*  Reset the device interface.*/
//	ret = ad7124_reset(dev);
	if (ret < 0)
		return ret;

	/* Update the device structure with power-on/reset settings */
	dev->check_ready = 1;

	/* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */
	for(reg_nr = AD7124_Status; (reg_nr < AD7124_Offset_0) && !(ret < 0);
	    reg_nr++) {
		if (dev->regs[reg_nr].rw == AD7124_RW) {
			ret = ad7124_write_register(dev, dev->regs[reg_nr]);
			if (ret < 0)
				break;
		}

		/* Get CRC State and device SPI interface settings */
		if (reg_nr == AD7124_Error_En) {
			ad7124_update_crcsetting(dev);
			ad7124_update_dev_spi_settings(dev);
		}
	}

	*device = dev;

	return ret;
}

/*************************************************************************************/

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

int32_t Ad7124Chip_toVoltage (uint32_t value, uint8_t gain, uint32_t vref, bool bipolar) {

  uint32_t aux1 =  value;
  uint32_t aux2 =  value;
  uint32_t voltage;

  aux1 = aux1 * 2;
  aux2 = aux2 / 2;

    voltage = (aux1 + aux2) *100000 / 0xFFFFFF;

  //voltage = voltage * vref / (double) gain;
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

uint32_t Ad7124Chip_read (uint8_t ch)
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
    Ad7124Chip_enableChannel (ch, TRUE);
    delay(100);
    Ad7124Chip_setMode (SingleConvMode);
    delay(100);
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
	ad7124_reset();


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

uint32_t Ad7124Chip_getData()
{
	uint8_t buffer[8];
	uint8_t i = 0;
	bool ready = false;


	  do {
	    /* Read the value of the Status Register */
			InitBuffer (buffer,0xAA);
			COMMAND = RD_REGISTER(AD7124_Status);
			i =	2;
			
			tx_n_rx(buffer, i); // This is not a standard function. You'll have to change it according to your uC. 

	    /* Check the RDY bit in the Status Register */
	    ready = (buffer[0] &
	             AD7124_STATUS_REG_RDY) == 0;

	  	  }	 while (!ready);


		InitBuffer (buffer,0xAA);
		COMMAND = RD_REGISTER(AD7124_Data);
		i =	4;

		tx_n_rx(buffer, i);

	return chnd_2_nmbr(buffer,0);
}

/***********************************************************************************/

int8_t spi_write_and_read (uint8_t * data, uint8_t len)
{
    Chip_SSP_DATA_SETUP_T xferConfig;
    uint8_t ret = 1;

    SSP_CS_LOW;

    uint8_t rx_aux[8]= {0,0,0,0,0,0};

	xferConfig.rx_cnt  = xferConfig.tx_cnt = 0;
	xferConfig.tx_data = data;
	xferConfig.rx_data = rx_aux;
	xferConfig.length  = len;

	ret = Chip_SSP_RWFrames_Blocking(LPC_SSP, &xferConfig);

	delay(100);

	SSP_CS_HIGH;

	data[0] = rx_aux[1];
	data[1] = rx_aux[2];
	data[2] = rx_aux[3];

	if (ret == 0)
		return COMM_ERR;
	else
		return (uint8_t)xferConfig.tx_cnt;

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

	Chip_GPIO_WritePortBit(LPCP0,SYNC_ADC , TRUE);
	Chip_GPIO_WritePortBit(LPCP0,CS_SPP , TRUE);

	/*************************************************************/
}

/***********************************************************************************/


void InitBuffer(unsigned char * buff,uint8_t fill)
{
	uint16_t i;

	for(i=0;i<8;i++)
			buff[i]=fill;
}


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



uint32_t chnd_2_nmbr (uint8_t *rxbuffer, uint8_t indice)
{
	uint32_t aux = 0;

	aux |= ((rxbuffer[indice] & 0xFF) << 16);
	aux |= ((rxbuffer[indice+1] & 0xFF) << 8);
	aux |= ((rxbuffer[indice+2] & 0xFF));

	return aux;
}

void delay (uint32_t time)
{
	//time = 1; 1us
	//time = 10; 2us
	//time = 100; 10us
	uint32_t i;
	for (i = 0; i <= time ; i++ );
	return;
}
