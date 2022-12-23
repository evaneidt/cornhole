/*
 * main.c
 *
 *  Created on: Oct 17, 2022
 *      Author: evanm
 */

#include <msp430.h> 
#include <stdlib.h>
#include "driverlib.h"
#include "hal.h"
#include "serial_data.h"
#include "LIS2HH12.h"
#include <stdint.h>

const uint8_t BOARD_ID = 0; // Change this when flashing a new board
                   	   	    // Even = Team 0; Odd = Team 1

volatile uint8_t periodic_flag_500ms = 0;
volatile uint8_t freefall_flag = 0;

volatile uint8_t FF_INT_EDGE = GPIO_LOW_TO_HIGH_TRANSITION;

#ifndef IO_PORT
#define IO_PORT         GPIO_PORT_P1
#endif
#ifndef INT1_PIN
#define INT1_PIN        GPIO_PIN4       // Inactivity interrupt pin
#endif
#ifndef INT2_PIN
#define INT2_PIN        GPIO_PIN6       // Free-Fall interrupt pin
#endif

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	__enable_interrupt();

	HAL_initPorts();
	HAL_initClocks(32768);

	HAL_initUARTA();
	HAL_enableUARTA();

	HAL_initI2C();
	HAL_enableI2C();
	HAL_I2C_setTransmitMode();

    configAccelerometer();

    //create interrupts
    //HAL_initInterrupts();

	while (1) {

	    if (periodic_flag_500ms) {
	        periodic_flag_500ms = 0;

	        // Send battery level every 500ms
	        sendBatteryData(0x0000);
	    }

	    if (freefall_flag) {
	    	freefall_flag = 0;

			Timer_B_stop(TIMER0_B_BASE);

	    	sendFreefallData(Timer_B_getCounterValue(TIMER0_B_BASE));

			Timer_B_clear(TIMER0_B_BASE);
	    }

        // Add delay?

	} // main while loop


} // main

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
	uint16_t INTPins = GPIO_getInterruptStatus(IO_PORT, INT1_PIN | INT2_PIN);

	if (INTPins & INT2_PIN) { // Free Fall interrupt

		if (FF_INT_EDGE) { // 1 = H-->L, end of detected free fall
			freefall_flag = 1;
		} else { // 0 = L-->H, start of detected free fall
			Timer_B_startCounter(TIMER0_B_BASE, TIMER_B_CONTINUOUS_MODE);
		}

		FF_INT_EDGE ^= 0x01; // Flip interrupt edge
		GPIO_selectInterruptEdge(IO_PORT, INT2_PIN, FF_INT_EDGE);
	}

	if (INTPins & INT1_PIN) { // Inactivity interrupt
		// TODO
	}

	GPIO_clearInterrupt(IO_PORT, INT1_PIN | INT2_PIN);

} // Port 1 ISR
