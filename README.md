## AD7124 24 BITS ADC

Implementation for a AD7124-4 24 bits sigma delta ADC. 


# Application Circuit
As a simple application circuit you can guide yourself with the following one:


This is a pressure sensor limited in bandwith (fc = 250Hz) connected directly to the AIN0 of the ADC. All the others inputs of the ADC were shorted to ground because they not used in this example. 

# Code

Before starting, please read the Disclaimer.

You'll need to include in your directory the following files:
- ad7124.c
- ad7124.h
- ad7124_regs.c
- ad7124_regs.h

You can find them in the "software" folder.

Setup:

Use the following lines to configure your AD7124

- Ad7124Chip_setConfig     (SETUP_0, RefInternal, Pga1, TRUE, BurnoutOff);
- Ad7124Chip_setChannel    (CHANNEL_0, SETUP_0, AIN0Input, AIN1Input, FALSE);
- Ad7124Chip_setAdcControl (StandbyMode, FullPower, TRUE, InternalClk);


Take a Sample:

uint32_t sample = Ad7124Chip_read(CHANNEL_0);

You can find a full working code for a LPC1769 Cortex®-M3 microcontroler in the folder "AD7124_MYFUNC"

## Disclaimer:

Please take in mind that all the functions in the file ad7124.c where modified to be used with a LPC1769 Cortex®-M3 microcontroler using not standard APIs.
This means that in order to make it work in your uC you'll need to set up your own SPI communication functions.

In this code, you'll need to adapt the functions:
  - Chip_SSP_RWFrames_Blocking() 
  - SSP_CS_LOW
  - SSP_CS_HIGH
