/*
 * LIS2HH12.h
 *
 *  Created on: Dec 20, 2022
 *      Author: evanm
 */

#ifndef CORE_LIS2HH12_H_
#define CORE_LIS2HH12_H_

#include <stdint.h>

// Register Addresses
#define TEMP_L 0x0B
#define TEMP_H 0x0C
#define WHO_AM_I 0x0F
#define ACT_THS 0x1E
#define ACT_DUR 0x1F
#define CTRL1 0x20
#define CTRL2 0x21
#define CTRL3 0x22
#define CTRL4 0x23
#define CTRL5 0x24
#define CTRL6 0x25
#define CTRL7 0x26
#define STATUS 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL 0x2E
#define FIFO_SRC 0x2F
#define IG_CFG1 0x30
#define IG_SRC1 0x31
#define IG_THS_X1 0x32
#define IG_THS_Y1 0x33
#define IG_THS_Z1 0x34
#define IG_DUR1 0x35
#define IG_CFG2 0x36
#define IG_SRC2 0x37
#define IG_THS2 0x38
#define IG_DUR2 0x39
#define XL_REFERENCE 0x3A
#define XH_REFERENCE 0x3B
#define YL_REFERENCE 0x3C
#define YH_REFERENCE 0x3D
#define ZL_REFERENCE 0x3E
#define ZH_REFERENCE 0x3F

#define CTRL1_VAL 0x3F				// X,Y,Z en, ODR = 100Hz, BDU en, High Res. dis

// Inactivity Values TODO Fix this
#define ACT_CTRL2_VAL 0x00			// Not used
#define ACT_CTRL3_VAL 0x20			// Inactivity Interrupt on INT1
#define ACT_CTRL4_VAL 0x00			// Not used
#define ACT_CTRL5_VAL 0x00			// Not used
#define ACT_CTRL6_VAL 0x00			// Not used
#define ACT_CTRL7_VAL 0x10			// INT1 not latched

// Free Fall Values
#define FF_CTRL2_VAL 0x00			// HPF disabled
#define FF_CTRL3_VAL 0x00			// Not used
#define FF_CTRL4_VAL 0x00			// Full-Scale = 2g
#define FF_CTRL5_VAL 0x00			// Interrupt active High, push-pull
#define FF_CTRL6_VAL 0x10			// Interrupt generator 2 on INT2
#define FF_CTRL7_VAL 0x20			// INT2 not latched, decrement duration counter
#define IG_THS2_VAL 0x2D			// Threshold = (2/256 * val) --> 0x2D = 352mg
#define IG_DUR2_VAL 0x83			// Wait en, Set 3 samples for event duration
#define IG_CFG2_VAL 0x95			// Logical AND of XLIE, YLIE, ZLIE --> FF detection

/*
 * Configure Accelerometer Registers
 */
void configAccelerometer(void);

/*
 * Send Slave Address, Register Subaddress, and Data
 */
void writeAccelRegister(uint8_t reg, uint8_t data);




#endif /* CORE_LIS2HH12_H_ */
