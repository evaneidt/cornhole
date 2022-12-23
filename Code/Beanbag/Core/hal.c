/*
 * hal.c
 *
 *  Created on: Oct 17, 2022
 *      Author: evanm
 */
#include <msp430.h>
#include "driverlib.h"
#include "hal.h"

void HAL_initPorts(void) {
   // Init outputs
#ifdef __MSP430_HAS_PORT1_R__
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
#endif

#ifdef __MSP430_HAS_PORT2_R__
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
#endif

    // Init inputs
       GPIO_setAsInputPin(GPIO_PORT_P1, INT1_PIN);
       GPIO_setAsInputPin(GPIO_PORT_P1, INT2_PIN);

       GPIO_setAsPeripheralModuleFunctionOutputPin(IO_PORT,
                                                   RF_TX_PIN,
                                                   GPIO_PRIMARY_MODULE_FUNCTION);

       GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, SDA_PIN | SCL_PIN);

       GPIO_setAsPeripheralModuleFunctionInputPin(IO_PORT,
                                                  SDA_PIN | SCL_PIN,
                                                  GPIO_PRIMARY_MODULE_FUNCTION);
}

void HAL_enable_RF_PWR(void) {
    GPIO_setOutputHighOnPin(IO_PORT, RF_PWR_PIN);
}

void HAL_disable_RF_PWR(void) {
    GPIO_setOutputLowOnPin(IO_PORT, RF_PWR_PIN);
}

void HAL_initClocks(uint32_t mclkFreq) {
    CS_setExternalClockSource(mclkFreq);

    CS_initClockSignal( CS_SMCLK,
                        CS_XT1CLK_SELECT,
                        CS_CLOCK_DIVIDER_1);
}

void HAL_initInterrupts(void) {
    GPIO_enableInterrupt(GPIO_PORT_P1, INT1_PIN);
    GPIO_enableInterrupt(GPIO_PORT_P1, INT2_PIN);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, INT1_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, INT2_PIN, FF_INT_EDGE);
}

void HAL_initTimers(void) {
	Timer_B_initContinuousModeParam timer0BConfig = {
		TIMER_B_CLOCKSOURCE_SMCLK,						// SMCLK Clock Source
		TIMER_B_CLOCKSOURCE_DIVIDER_1,					// Clock Source Divider = 1
		TIMER_B_TBIE_INTERRUPT_DISABLE,					// Disable TIMER_B interrupts
		TIMER_B_DO_CLEAR,								// Clear Timer
		0												// 0 = don't start timer
	}; // Timer0 B Config

	Timer_B_initContinuousMode(TIMER0_B_BASE, &timer0BConfig);
}

/*
 *  UART
 */

void HAL_initUARTA(void) {
    EUSCI_A_UART_initParam uartConfig = {
        /*  Baudrate = 9600, clock freq = 32kHz
             * UCBRx = 3, UCBRFx = 0, UCBRSx = 0x92, UCOS16 = 0
         */
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
        3,                                              // Clock Prescalar
        0,                                              // First Modulation Register
        0x92,                                           // Second Modulation Register
        EUSCI_A_UART_EVEN_PARITY,                       // Even Parity
        EUSCI_A_UART_MSB_FIRST,                         // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // 1 Stop Bit
        EUSCI_A_UART_MODE,                              // UART Operation Mode
        EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Baud Generation
    }; // UART Config

    EUSCI_A_UART_init(USCI_A_BASE, &uartConfig);
}

void HAL_enableUARTA(void) {
    EUSCI_A_UART_enable(USCI_A_BASE);
}

void HAL_disableUARTA(void) {
    EUSCI_A_UART_disable(USCI_A_BASE);
}

void HAL_UARTA_transmitData(const uint8_t *dataBuffer, size_t length) {
	size_t i = 0;
	for (; i < length; ++i) {
	    EUSCI_A_UART_transmitData(USCI_A_BASE, dataBuffer[i]);
	}
}

bool HAL_UARTA_queryBusy(void) {
    return EUSCI_A_UART_queryStatusFlags(USCI_A_BASE, EUSCI_A_UART_BUSY)==EUSCI_A_UART_BUSY;
}

void HAL_UARTA_dormant(void) {
    EUSCI_A_UART_setDormant(USCI_A_BASE);
}

void HAL_UARTA_wake(void) {
    EUSCI_A_UART_resetDormant(USCI_A_BASE);
}

/*
 *  I2C
 */

void HAL_initI2C(void) {
    EUSCI_B_I2C_initMasterParam i2cConfig = {
            EUSCI_B_I2C_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
            CS_getSMCLK(),                              // SMCLK 32kHz
            EUSCI_B_I2C_SET_DATA_RATE_100KBPS,          // I2C Clock
            0,                                          // No byte counter threshold
            EUSCI_B_I2C_NO_AUTO_STOP                    // No Autostop
    }; // I2C Config

    EUSCI_B_I2C_initMaster(USCI_B_BASE, &i2cConfig);
    EUSCI_B_I2C_setSlaveAddress(USCI_B_BASE, ACCEL_ADDR);
}

void HAL_enableI2C(void) {
    EUSCI_B_I2C_enable(USCI_B_BASE);
}

void HAL_disableI2C(void) {
    EUSCI_B_I2C_disable(USCI_B_BASE);
}

void HAL_I2C_setTransmitMode(void) {
    EUSCI_B_I2C_setMode(USCI_B_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
}

void HAL_I2C_setReceiveMode(void) {
    EUSCI_B_I2C_setMode(USCI_B_BASE, EUSCI_B_I2C_RECEIVE_MODE);
}


