/*
===============================================================================
 Name        : Ex2_FreeRtos.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "board.h"
#include "ad7124.h"
#include <cr_section_macros.h>


void prueba (void);

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

#define mainDELAY_LOOP_COUNT		( 0xfffff )



/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */

static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();
	GPIOs_Init();
	SPI_Init();
	AD7124_Init();
}


void AD7124_Init (void)
{
	uint32_t aux = 0;
	uint32_t value = 0;
	uint32_t voltage = 0;
	uint8_t buffer[8];
	uint8_t i = 0;
	bool ready = false;


	// Setting the configuration 0:
	// - use of the internal reference voltage 2.5V
	// - gain of 1 for a bipolar measurement +/- 2.5V
	aux = Ad7124Chip_setConfig 	   (SETUP_0, RefInternal, Pga1, TRUE, BurnoutOff);
  	// Setting channel 0 using pins AIN1(+)/AIN0(-)
	aux = Ad7124Chip_setChannel    (CHANNEL_0, SETUP_0, AIN0Input, AIN1Input, FALSE); //revisar este último false!!
  	// Configuring ADC in Full Power Mode (Fastest)
	aux = Ad7124Chip_setAdcControl (StandbyMode, FullPower, TRUE, InternalClk);

	while (1)
	{

		value = Ad7124Chip_read(CHANNEL_0);


			value = chnd_2_nmbr(buffer,0);
			voltage = value;
		    voltage = ((voltage*1000) / 0x7FFFFFUL) - 1;
		    voltage = (voltage*25) / (10 * 1);

	}

}




/*-----------------------------------------------------------*/
int main( void )
{

	prvSetupHardware();

	xTaskCreate( vTaskFunction, "Task 1", 240, "Task 1 is running\n", 1, NULL );
	vTaskStartScheduler();

	for( ;; );
	return 0;
}

/*-----------------------------------------------------------*/


void vTaskFunction( void *pvParameters )
{


	for( ;; )
	{


		vTaskDelay(1000/portTICK_PERIOD_MS);

	}
}
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}
