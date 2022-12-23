/*
 * serial_data.c
 *
 *  Created on: Dec 20, 2022
 *      Author: evanm
 */

#include "serial_data.h"
#include "hal.h"

void sendBatteryData(uint16_t batteryValue) {
	// Send 2 byte battery level

	sendPacket(batteryValue, BATTERY_PACKET);

} // Send Battery Data

void sendFreefallData(uint16_t timerValue) {
	// Send 2 byte elapsed free fall time

	sendPacket(timerValue, FREEFALL_PACKET);

} // Send Freefall Data

void sendPacket(uint16_t value, uint8_t packetType) {
	HAL_enable_RF_PWR();

	uint8_t inputData[2] = {(value & 0xFF00) >> 8, value & 0xFF}; // "Big Endian"
	uint8_t outputData[5];

	encodeCOBS(inputData, 2, outputData);

	outputData[0] = START_BIT | DEST_ID;
	outputData[1] = outputData[1] | (BOARD_ID << 1) | packetType;

	HAL_UARTA_transmitData(outputData, 5);

	while(HAL_UARTA_queryBusy());

	HAL_disable_RF_PWR();
} // Send Packet

void encodeCOBS(const uint8_t *inputBuffer, size_t length, uint8_t *outputBuffer) {
	// TODO
}
