/*
 * serial_data.h
 *
 *  Created on: Dec 20, 2022
 *      Author: evanm
 */

#ifndef CORE_SERIAL_DATA_H_
#define CORE_SERIAL_DATA_H_

#include <stdint.h>
#include <stddef.h>

extern const uint8_t BOARD_ID;

#define START_BIT 0x80
#define DEST_ID 0x69

#define BATTERY_PACKET 0x0
#define FREEFALL_PACKET 0x1

#define STOP_BYTE 0x00

// Input: 2 byte battery level
void sendBatteryData(uint16_t batteryValue);

// Input: 2 byte elapsed free fall time
void sendFreefallData(uint16_t timerValue);

void sendPacket(uint16_t value, uint8_t packetType);

void encodeCOBS(const uint8_t *inputBuffer, size_t length, uint8_t *outputBuffer);

#endif /* CORE_SERIAL_DATA_H_ */
