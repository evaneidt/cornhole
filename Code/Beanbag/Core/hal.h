/*
 * hal.h
 *
 *  Created on: Oct 17, 2022
 *      Author: evanm
 */

#ifndef CORE_HAL_H_
#define CORE_HAL_H_

#include "driverlib.h"
#include <stdint.h>
#include <stddef.h>

extern volatile uint8_t FF_INT_EDGE;

#define IO_PORT         GPIO_PORT_P1
#define INT1_PIN        GPIO_PIN4       // Inactivity interrupt pin
#define INT2_PIN        GPIO_PIN6       // Free-Fall interrupt pin
#define SDA_PIN         GPIO_PIN2       // eUSCI B
#define SCL_PIN         GPIO_PIN3       // eUSCI B
#define RF_PWR_PIN      GPIO_PIN5
#define RF_TX_PIN       GPIO_PIN7

#define USCI_A_BASE 0x500
#define USCI_B_BASE 0x540

#define TIMER0_B_BASE 0x380

#define ACCEL_ADDR 0x1E

void HAL_initPorts(void);
void HAL_initClocks(uint32_t mclkFreq);
void HAL_initInterrupts(void);
void HAL_initTimers(void);

void HAL_enable_RF_PWR(void);
void HAL_disable_RF_PWR(void);

// UART
void HAL_initUARTA(void);
void HAL_enableUARTA(void);
void HAL_disableUARTA(void);
void HAL_UARTA_transmitData(const uint8_t* dataBuffer, size_t length);
bool HAL_UARTA_queryBusy(void);
void HAL_UARTA_dormant(void);
void HAL_UARTA_wake(void);

// I2C
void HAL_initI2C(void);
void HAL_enableI2C(void);
void HAL_disableI2C(void);
void HAL_I2C_setTransmitMode(void);
void HAL_I2C_setReceiveMode(void);

#endif /* CORE_HAL_H_ */
