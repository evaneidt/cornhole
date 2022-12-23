/*
 * LIS2HH12.c
 *
 *  Created on: Dec 20, 2022
 *      Author: evanm
 */

#include "LIS2HH12.h"
#include "hal.h"

void configAccelerometer(void) {

	writeAccelRegister(CTRL1, CTRL1_VAL);

	writeAccelRegister(CTRL2, FF_CTRL2_VAL | ACT_CTRL2_VAL);
	writeAccelRegister(CTRL3, FF_CTRL3_VAL | ACT_CTRL3_VAL);
	writeAccelRegister(CTRL4, FF_CTRL4_VAL | ACT_CTRL4_VAL);
	writeAccelRegister(CTRL5, FF_CTRL5_VAL | ACT_CTRL5_VAL);
	writeAccelRegister(CTRL6, FF_CTRL6_VAL | ACT_CTRL6_VAL);
	writeAccelRegister(CTRL7, FF_CTRL7_VAL | ACT_CTRL7_VAL);

	// FF Registers
	writeAccelRegister(IG_THS2, IG_THS2_VAL);
	writeAccelRegister(IG_DUR2, IG_DUR2_VAL);
	writeAccelRegister(IG_CFG2, IG_CFG2_VAL);

	// ACT Registers
	// TODO Fix this


	// ***Note: To implement activity, might have to make interrupts active low
	// In this case, look into making X/Y/ZLIE --> X/Y/ZHIE and changing the INT AND condition
	// Also, interrupts are EDGE selectable and dir. independent for INT1 & INT2 so this may not be needed
}

void writeAccelRegister(uint8_t reg, uint8_t data) {
    // Send slave address
    EUSCI_B_I2C_masterSendMultiByteStart(USCI_B_BASE, ACCEL_ADDR);

    // Send register subaddress
    EUSCI_B_I2C_masterSendMultiByteNext(USCI_B_BASE, reg);

    // Send data
    EUSCI_B_I2C_masterSendMultiByteFinish(USCI_B_BASE, data);
}
