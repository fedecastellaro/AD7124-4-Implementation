/*
 * ad7124.h
 *
 *  Created on: 1 de oct. de 2018
 *      Author: root
 */

#ifndef AD7124_H_
#define AD7124_H_

/***************************************************************************//**
*   @file    AD7124.h
*   @brief   AD7124 header file.
*   @devices AD7124-4, AD7124-8
*
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "ad7124_regs.h"
#include "stdbool.h"
#include "board.h"
#include "chip.h"


/*-------------DEFINICIONES SPI---------------------*/


#define BUFFER_SIZE_SSP		100

#define MOSI_SPP			0,18
#define MISO_SPP			0,17
#define SCK_SPP				0,15
#define CS_SPP				0,16
#define SYNC_ADC			0,23
#define LPC_SSP				LPC_SSP0

#define BUTTON_START		2,11
#define BUTTON_STOP			2,12

/*
#define MOSI_SPP			0,18
#define MISO_SPP			0,17
#define SCK_SPP				0,15
#define CS_SPP				0,23
#define LPC_SSP				LPC_SSP0
#define SYNC_ADC			2,13*/

#define LPCP0 				LPC_GPIO

#define TX_BUFFER_SIZE		10

volatile uint8_t Tx_Buf[10];
volatile uint8_t Rx_Buf[10];


/******************************************************************************/
/******************* Register map and register definitions ********************/
/******************************************************************************/

#define	AD7124_RW 1   /* Read and Write */
#define	AD7124_R  2   /* Read only */
#define AD7124_W  3   /* Write only */


/* AD7124 Register Map */
#define AD7124_COMM_REG      0x00
#define AD7124_STATUS_REG    0x00
#define AD7124_ADC_CTRL_REG  0x01
#define AD7124_DATA_REG      0x02
#define AD7124_IO_CTRL1_REG  0x03
#define AD7124_IO_CTRL2_REG  0x04
#define AD7124_ID_REG        0x05
#define AD7124_ERR_REG       0x06
#define AD7124_ERREN_REG     0x07
#define AD7124_CH0_MAP_REG   0x09
#define AD7124_CH1_MAP_REG   0x0A
#define AD7124_CH2_MAP_REG   0x0B
#define AD7124_CH3_MAP_REG   0x0C
#define AD7124_CH4_MAP_REG   0x0D
#define AD7124_CH5_MAP_REG   0x0E
#define AD7124_CH6_MAP_REG   0x0F
#define AD7124_CH7_MAP_REG   0x10
#define AD7124_CH8_MAP_REG   0x11
#define AD7124_CH9_MAP_REG   0x12
#define AD7124_CH10_MAP_REG  0x13
#define AD7124_CH11_MAP_REG  0x14
#define AD7124_CH12_MAP_REG  0x15
#define AD7124_CH13_MAP_REG  0x16
#define AD7124_CH14_MAP_REG  0x17
#define AD7124_CH15_MAP_REG  0x18
#define AD7124_CFG0_REG      0x19
#define AD7124_CFG1_REG      0x1A
#define AD7124_CFG2_REG      0x1B
#define AD7124_CFG3_REG      0x1C
#define AD7124_CFG4_REG      0x1D
#define AD7124_CFG5_REG      0x1E
#define AD7124_CFG6_REG      0x1F
#define AD7124_CFG7_REG      0x20
#define AD7124_FILT0_REG     0x21
#define AD7124_FILT1_REG     0x22
#define AD7124_FILT2_REG     0x23
#define AD7124_FILT3_REG     0x24
#define AD7124_FILT4_REG     0x25
#define AD7124_FILT5_REG     0x26
#define AD7124_FILT6_REG     0x27
#define AD7124_FILT7_REG     0x28
#define AD7124_OFFS0_REG     0x29
#define AD7124_OFFS1_REG     0x2A
#define AD7124_OFFS2_REG     0x2B
#define AD7124_OFFS3_REG     0x2C
#define AD7124_OFFS4_REG     0x2D
#define AD7124_OFFS5_REG     0x2E
#define AD7124_OFFS6_REG     0x2F
#define AD7124_OFFS7_REG     0x30
#define AD7124_GAIN0_REG     0x31
#define AD7124_GAIN1_REG     0x32
#define AD7124_GAIN2_REG     0x33
#define AD7124_GAIN3_REG     0x34
#define AD7124_GAIN4_REG     0x35
#define AD7124_GAIN5_REG     0x36
#define AD7124_GAIN6_REG     0x37
#define AD7124_GAIN7_REG     0x38

/* Communication Register bits */
#define AD7124_COMM_REG_WEN    (0 << 7)
#define AD7124_COMM_REG_WR     (0 << 6)
#define AD7124_COMM_REG_RD     (1 << 6)
#define AD7124_COMM_REG_RA(x)  ((x) & 0x3F)

/* Status Register bits */
#define AD7124_STATUS_REG_RDY          (1 << 7)
#define AD7124_STATUS_REG_ERROR_FLAG   (1 << 6)
#define AD7124_STATUS_REG_POR_FLAG     (1 << 4)
#define AD7124_STATUS_REG_CH_ACTIVE(x) ((x) & 0xF)

/* ADC_Control Register bits */
#define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL   (1 << 12)
#define AD7124_ADC_CTRL_REG_CONT_READ      (1 << 11)
#define AD7124_ADC_CTRL_REG_DATA_STATUS    (1 << 10)
#define AD7124_ADC_CTRL_REG_CS_EN          (1 << 9)
#define AD7124_ADC_CTRL_REG_REF_EN         (1 << 8)
#define AD7124_ADC_CTRL_REG_POWER_MODE(x)  (((x) & 0x3) << 6)
#define AD7124_ADC_CTRL_REG_MODE(x)        (((x) & 0xF) << 2)
#define AD7124_ADC_CTRL_REG_CLK_SEL(x)     (((x) & 0x3) << 0)

/* IO_Control_1 Register bits */
#define AD7124_IO_CTRL1_REG_GPIO_DAT2     (1 << 23)
#define AD7124_IO_CTRL1_REG_GPIO_DAT1     (1 << 22)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL2    (1 << 19)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL1    (1 << 18)
#define AD7124_IO_CTRL1_REG_PDSW          (1 << 15)
#define AD7124_IO_CTRL1_REG_IOUT1(x)      (((x) & 0x7) << 11)
#define AD7124_IO_CTRL1_REG_IOUT0(x)      (((x) & 0x7) << 8)
#define AD7124_IO_CTRL1_REG_IOUT_CH1(x)   (((x) & 0xF) << 4)
#define AD7124_IO_CTRL1_REG_IOUT_CH0(x)   (((x) & 0xF) << 0)

/* IO_Control_1 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT4     (1 << 23)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT3     (1 << 22)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT2     (1 << 21)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT1     (1 << 20)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL4    (1 << 19)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL3    (1 << 18)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL2    (1 << 17)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL1    (1 << 16)

/* IO_Control_2 Register bits */
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 15)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 14)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 11)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 10)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 5)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 4)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

/* IO_Control_2 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15  (1 << 15)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14  (1 << 14)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13  (1 << 13)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12  (1 << 12)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11  (1 << 11)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10  (1 << 10)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9   (1 << 9)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8   (1 << 8)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 7)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 6)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 5)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 4)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 3)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 2)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

/* ID Register bits */
#define AD7124_ID_REG_DEVICE_ID(x)   (((x) & 0xF) << 4)
#define AD7124_ID_REG_SILICON_REV(x) (((x) & 0xF) << 0)

/* Error Register bits */
#define AD7124_ERR_REG_LDO_CAP_ERR        (1 << 19)
#define AD7124_ERR_REG_ADC_CAL_ERR        (1 << 18)
#define AD7124_ERR_REG_ADC_CONV_ERR       (1 << 17)
#define AD7124_ERR_REG_ADC_SAT_ERR        (1 << 16)
#define AD7124_ERR_REG_AINP_OV_ERR        (1 << 15)
#define AD7124_ERR_REG_AINP_UV_ERR        (1 << 14)
#define AD7124_ERR_REG_AINM_OV_ERR        (1 << 13)
#define AD7124_ERR_REG_AINM_UV_ERR        (1 << 12)
#define AD7124_ERR_REG_REF_DET_ERR        (1 << 11)
#define AD7124_ERR_REG_DLDO_PSM_ERR       (1 << 9)
#define AD7124_ERR_REG_ALDO_PSM_ERR       (1 << 7)
#define AD7124_ERR_REG_SPI_IGNORE_ERR     (1 << 6)
#define AD7124_ERR_REG_SPI_SLCK_CNT_ERR   (1 << 5)
#define AD7124_ERR_REG_SPI_READ_ERR       (1 << 4)
#define AD7124_ERR_REG_SPI_WRITE_ERR      (1 << 3)
#define AD7124_ERR_REG_SPI_CRC_ERR        (1 << 2)
#define AD7124_ERR_REG_MM_CRC_ERR         (1 << 1)

/* Error_En Register bits */
#define AD7124_ERREN_REG_MCLK_CNT_EN           (1 << 22)
#define AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN   (1 << 21)
#define AD7124_ERREN_REG_LDO_CAP_CHK(x)        (((x) & 0x3) << 19)
#define AD7124_ERREN_REG_ADC_CAL_ERR_EN        (1 << 18)
#define AD7124_ERREN_REG_ADC_CONV_ERR_EN       (1 << 17)
#define AD7124_ERREN_REG_ADC_SAT_ERR_EN        (1 << 16)
#define AD7124_ERREN_REG_AINP_OV_ERR_EN        (1 << 15)
#define AD7124_ERREN_REG_AINP_UV_ERR_EN        (1 << 14)
#define AD7124_ERREN_REG_AINM_OV_ERR_EN        (1 << 13)
#define AD7124_ERREN_REG_AINM_UV_ERR_EN        (1 << 12)
#define AD7124_ERREN_REG_REF_DET_ERR_EN        (1 << 11)
#define AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN (1 << 10)
#define AD7124_ERREN_REG_DLDO_PSM_ERR_ERR      (1 << 9)
#define AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN (1 << 8)
#define AD7124_ERREN_REG_ALDO_PSM_ERR_EN       (1 << 7)
#define AD7124_ERREN_REG_SPI_IGNORE_ERR_EN     (1 << 6)
#define AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN   (1 << 5)
#define AD7124_ERREN_REG_SPI_READ_ERR_EN       (1 << 4)
#define AD7124_ERREN_REG_SPI_WRITE_ERR_EN      (1 << 3)
#define AD7124_ERREN_REG_SPI_CRC_ERR_EN        (1 << 2)
#define AD7124_ERREN_REG_MM_CRC_ERR_EN         (1 << 1)

/* Channel Registers 0-15 bits */
#define AD7124_CH_MAP_REG_CH_ENABLE    (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x)      (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x)      (((x) & 0x1F) << 0)

/* Configuration Registers 0-7 bits */
#define AD7124_CFG_REG_BIPOLAR     (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x)  (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP    (1 << 8)
#define AD7124_CFG_REG_REF_BUFM    (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP    (1 << 6)
#define AD7124_CFG_REG_AINN_BUFM   (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x)  ((x) & 0x3) << 3
#define AD7124_CFG_REG_PGA(x)      (((x) & 0x7) << 0)

/* Filter Register 0-7 bits */
#define AD7124_FILT_REG_FILTER(x)         (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60             (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x)    (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE      (1 << 16)
#define AD7124_FILT_REG_FS(x)             (((x) & 0x7FF) << 0)


#define SSP_CS_LOW        Chip_GPIO_WritePortBit(LPC_GPIO, CS_SPP, FALSE)
#define SSP_CS_HIGH       Chip_GPIO_WritePortBit(LPC_GPIO, CS_SPP, TRUE)
#define SETUP_0			  0

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

typedef enum OperatingMode {
  ContinuousMode = 0, /**< Continuous conversion mode (default). In continuous conversion mode, the ADC continuously performs conversions and places the result in the data register. */
  SingleConvMode, /**< Single conversion mode. When single conversion mode is selected, the ADC powers up and performs a single conversion on the selected channel. */
  StandbyMode, /**< Standby mode. In standby mode, all sections of the AD7124 can be powered down except the LDOs. */
  PowerDownMode, /**< Power-down mode. In power-down mode, all the AD7124 circuitry is powered down, including the current sources, power switch, burnout currents, bias voltage generator, and clock circuitry. */
  IdleMode, /**< Idle mode. In idle mode, the ADC filter and modulator are held in a reset state even though the modulator clocks continue to be provided. */
  InternalOffsetCalibrationMode, /**< Internal zero-scale (offset) calibration. An internal short is automatically connected to the input. RDY goes high when the calibration is initiated and returns low when the calibration is complete. */
  InternalGainCalibrationMode, /**< Internal full-scale (gain) calibration. A full-scale input voltage is automatically connected to the selected analog input for this calibration. */
  SystemOffsetCalibrationMode, /**< System zero-scale (offset) calibration. Connect the system zero-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete. */
  SystemGainCalibrationMode /**< System full-scale (gain) calibration. Connect the system full-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete. */
}OperatingMode;
/**
 * @enum PowerMode
 * @brief Power Mode Select
 * These bits select the power mode. The current consumption and output data rate
 * ranges are dependent on the power mode.
 */
typedef enum PowerMode {
  LowPower = 0, /**< low power */
  MidPower, /**< mid power */
  FullPower /**< full power */
}PowerMode;
/**
 * @enum ClkSel
 * @brief These bits select the clock source for the ADC
 * Either the on-chip 614.4 kHz clock can be used or an external clock can be
 * used. The ability to use an external clock allows several AD7124 devices to be
 * synchronized. Also, 50 Hz and 60 Hz rejection is improved when an accurate external clock drives the ADC.
 */
typedef enum ClkSel {
  InternalClk = 0, /**< internal 614.4 kHz clock. The internal clock is not available at the CLK pin. */
  InternalWithOutputClk, /**< internal 614.4 kHz clock. This clock is available at the CLK pin. */
  ExternalClk, /**< external 614.4 kHz clock. */
  ExternalDiv4Clk /**< external clock. The external clock is divided by 4 within the AD7124. */
}ClkSel;
/**
 * @enum IoutCurrent
 * @brief These bits set the value of the excitation current for IOUT
 */
typedef enum IoutCurrent {
  CurrentOff = 0, /**< Off */
  Current50uA, /**< 50 μA */
  Current100uA, /**< 100 μA */
  Current250uA, /**< 250 μA */
  Current500uA, /**< 500 μA */
  Current750uA, /**< 750 μA */
  Current1000uA /**< 1 mA */
}IoutCurrent;
/**
 * @enum IoutCh
 * @brief Channel select bits for the excitation current for IOUT.
 */
typedef enum IoutCh {
  IoutCh0 = 0, /**< IOUT is available on the AIN0 pin. */
  IoutCh1 = 1, /**< IOUT is available on the AIN1 pin. */
  IoutCh2 = 4, /**< IOUT is available on the AIN2 pin. */
  IoutCh3 = 5, /**< IOUT is available on the AIN3 pin. */
  IoutCh4 = 10, /**< IOUT is available on the AIN4 pin. */
  IoutCh5 = 11, /**< IOUT is available on the AIN5 pin. */
  IoutCh6 = 14, /**< IOUT is available on the AIN6 pin. */
  IoutCh7 = 15 /**< IOUT is available on the AIN7 pin. */
}IoutCh;
/**
 * @enum InputSel
 * @brief Analog input AIN input select
 */
typedef enum InputSel {
  AIN0Input = 0, /**< AIN0 */
  AIN1Input, /**< AIN1 */
  AIN2Input, /**< AIN2 */
  AIN3Input, /**< AIN3 */
  AIN4Input, /**< AIN4 */
  AIN5Input, /**< AIN5 */
  AIN6Input, /**< AIN6 */
  AIN7Input, /**< AIN7 */
  TEMPInput = 16, /**< Temperature sensor (internal) */
  AVSSInput, /**< AVss */
  REFInput, /**< Internal reference */
  DGNDInput, /**< DGND. */
  AVDD6PInput, /**< (AVdd − AVss)/6+. Use in conjunction with (AVdd − AVss)/6− to monitor supply AVdd − AVss . */
  AVDD6MInput, /**< (AVdd − AVss)/6−. Use in conjunction with (AVdd − AVss)/6+ to monitor supply AVdd − AVss . */
  IOVDD6PInput, /**< (IOVdd − DGND)/6+. Use in conjunction with (IOVdd − DGND)/6− to monitor IOVdd − DGND. */
  IOVDD6MInput, /**< (IOVdd − DGND)/6−. Use in conjunction with (IOVdd − DGND)/6+ to monitor IOVdd − DGND. */
  ALDO6PInput, /**< (ALDO − AVss)/6+. Use in conjunction with (ALDO − AVss)/6− to monitor the analog LDO. */
  ALDO6MInput, /**< (ALDO − AVss)/6−. Use in conjunction with (ALDO − AVss)/6+ to monitor the analog LDO. */
  DLDO6PInput, /**< (DLDO − DGND)/6+. Use in conjunction with (DLDO − DGND)/6− to monitor the digital LDO. */
  DLDO6MInput, /**< (DLDO − DGND)/6−. Use in conjunction with (DLDO − DGND)/6+ to monitor the digital LDO. */
  V20mVPInput, /**< V_20MV_P. Use in conjunction with V_20MV_M to apply a 20 mV p-p signal to the ADC. */
  V20mVMInput /**< V_20MV_M. Use in conjunction with V_20MV_P to apply a 20 mV p-p signal to the ADC. */
}InputSel;
/**
 * @enum PgaSel
 * @brief Gain select bits.
 * These bits select the gain to use when converting on any channels using this configuration register.
 */
typedef enum PgaSel {
  Pga1 = 0, /**< Gain 1, Input Range When VREF = 2.5 V: ±2.5 V */
  Pga2, /**< Gain 2, Input Range When VREF = 2.5 V: ±1.25 V */
  Pga4, /**< Gain 4, Input Range When VREF = 2.5 V: ± 625 mV */
  Pga8, /**< Gain 8, Input Range When VREF = 2.5 V: ±312.5 mV */
  Pga16, /**< Gain 16, Input Range When VREF = 2.5 V: ±156.25 mV */
  Pga32, /**< Gain 32, Input Range When VREF = 2.5 V: ±78.125 mV */
  Pga64, /**< Gain 64, Input Range When VREF = 2.5 V: ±39.06 mV */
  Pga128 /**< Gain 128, Input Range When VREF = 2.5 V: ±19.53 mV */
}PgaSel;
/**
 * @enum RefSel
 * @brief Reference source select bits.
 * These bits select the reference source to use when converting on any channels using
 * this configuration register.
 */
typedef enum RefSel {
  RefIn1 = 0, /**< REFIN1(+)/REFIN1(−). */
  RefIn2, /**< REFIN2(+)/REFIN2(−). */
  RefInternal, /**< internal reference. */
  RefAVdd /**< AVDD */
}RefSel;
/**
 * @enum BurnoutCurrent
 * @brief These bits select the magnitude of the sensor burnout detect current source.
 */
typedef enum BurnoutCurrent {
  BurnoutOff = 0, /**< burnout current source off (default). */
  Burnout500nA, /**< burnout current source on, 0.5 μA. */
  Burnout2uA, /**< burnout current source on, 2 μA. */
  Burnout4uA /**< burnout current source on, 4 μA. */
}BurnoutCurrent;
/**
 * @enum FilterType
 * @brief Filter type select bits.
 * These bits select the filter type.
 */
typedef enum FilterType {
  Sinc4Filter = 0, /**< sinc4 filter (default). */
  Sinc3Filter = 2, /**< sinc 3 filter. */
  Sinc4FastFilter = 4, /**< fast settling filter using the sinc 4 filter. The sinc 4 filter is followed by an averaging block, which results in a settling time equal to the conversion time. In full power and mid power modes, averaging by 16 occurs whereas averaging by 8 occurs in low power mode. */
  Sinc3FastFilter = 5, /**< fast settling filter using the sinc 3 filter. The sinc 3 filter is followed by an averaging block, which results in a settling time equal to the conversion time. In full power and mid power modes, averaging by 16 occurs whereas averaging by 8 occurs in low power mode. */
  PostFilter = 7 /**< post filter enabled. The AD7124 includes several post filters, selectable using the POST_FILTER bits. The post filters have single cycle settling, the settling time being considerably better than a simple sinc 3 /sinc 4 filter. These filters offer excellent 50 Hz and60 Hz rejection. */
}FilterType;
/**
 * @enum PostFilterType
 * @brief Post filter type select bits.
 * When the filter bits are set to 1, the sinc 3 filter is followed by a post filter which
 * offers good 50 Hz and 60 Hz rejection at output data rates that have zero latency approximately.
 */
typedef enum PostFilterType {
  NoPostFilter   = 0, /**< No Post Filter (Default value) */
  dB47PostFilter = 2, /**< Rejection at 50 Hz and 60 Hz ± 1 Hz: 47 dB, Output Data Rate (SPS): 27.27 Hz */
  dB62PostFilter = 3, /**< Rejection at 50 Hz and 60 Hz ± 1 Hz: 62 dB, Output Data Rate (SPS): 25 Hz */
  dB86PostFilter = 5, /**< Rejection at 50 Hz and 60 Hz ± 1 Hz: 86 dB, Output Data Rate (SPS): 20 Hz */
  dB92PostFilter = 6 /**< Rejection at 50 Hz and 60 Hz ± 1 Hz: 92 dB, Output Data Rate (SPS): 16.7 Hz */
}PostFilterType;

/*
 * The structure describes the device and is used with the ad7124 driver.
 * @spi_desc: A reference to the SPI configuration of the device.
 * @regs: A reference to the register list of the device that the user must
 *       provide when calling the Setup() function.
 * @userCRC: Whether to do or not a cyclic redundancy check on SPI transfers.
 * @check_ready: When enabled all register read and write calls will first wait
 *               until the device is ready to accept user requests.
 * @spi_rdy_poll_cnt: Number of times the driver should read the Error register
 *                    to check if the device is ready to accept user requests,
 *                    before a timeout error will be issued.
 */

struct ad7124_device {

	/* Device Settings */
	struct ad7124_st_reg	*regs;
	int16_t use_crc;
	int16_t check_ready;
	int16_t spi_rdy_poll_cnt;
};

struct ad7124_init_param {

	/* Device Settings */
	struct ad7124_st_reg	*regs;
	int16_t spi_rdy_poll_cnt;
};

/******************************************************************************/
/******************* AD7124 Constants *****************************************/
/******************************************************************************/
#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
#define AD7124_DISABLE_CRC 0
#define AD7124_USE_CRC 1
#define TIMEOUT_AD7124		100

#define RD_REGISTER(x)	( ((x)&(0x3F)) | (1<<6) )
#define WR_REGISTER(x)	( ((x)&(0x3F)) & ~(1<<6) )
#define COMMAND			buffer[0]
#define BYTE_1			buffer[1]
#define BYTE_2			buffer[2]
#define BYTE_3			buffer[3]
#define BYTE_4			buffer[4]

#define DATA_1			buffer[0]
#define DATA_2			buffer[1]
#define DATA_3			buffer[2]
#define DATA_4			buffer[3]
#define DATA_5			buffer[4]

#define CHANNEL_0		0
/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

int  AD7124_Init(void);
void SPI_Init	(void);
void GPIOs_Init (void);

/************************ Added by me ***************************/
long 	 	chnd_2_nmbr 					  (uint8_t *rxbuffer, uint8_t indice);
void 	 	tx_n_rx						   	  (uint8_t *data, uint8_t length );

uint32_t 	Ad7124Chip_setAdcControl 		  (OperatingMode mode, PowerMode power_mode, bool ref_en, ClkSel clk_sel);
int32_t 	Ad7124Chip_setConfig 			  (uint8_t cfg, RefSel ref, PgaSel pga, bool bipolar, BurnoutCurrent burnout);
uint32_t 	Ad7124Chip_setChannel 			  (uint8_t ch, uint8_t cfg, InputSel ainp, InputSel ainm, bool enable);
long 		Ad7124Chip_read 				  (uint8_t ch);
void		Ad7124_begin					  (void);
void  		Ad7124Chip_startSingleConversion  (uint8_t ch);
void  		Ad7124Chip_enableChannel 		  (uint8_t ch, bool enable);
void	  	Ad7124Chip_setMode 			  	  (OperatingMode mode);

long 		Ad7124Chip_getData				  (void);

double 		Ad7124Chip_toVoltage 			  (long value, int gain, double vref, bool bipolar);
void 		Ad7124Chip_Offset_Gain_Calibration(uint32_t *offset,uint32_t *gain);
uint8_t 	Ad7124Chip_ReadID				  (void);
int8_t  	spi_write_and_read 				  (uint8_t * data, uint8_t len);
uint8_t 	spi_write 						  (uint8_t * data, uint8_t len);
uint8_t 	spi_read 						  (uint8_t * data, uint8_t len);
uint8_t 	spiwrite						  (uint8_t c);

double		Ad7124Chip_toPressure			  (double voltage);
double 		toPressure 						  (double voltage );
int 		AD7124_Init 					  (void);

void	 	delay 							  (uint32_t time);

void InitBuffer(unsigned char *,uint8_t);

/* The task functions. */
void vTaskFunction( void *pvParameters );

#endif /* AD7124_H_ */
