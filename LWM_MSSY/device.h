#pragma once
#ifndef DEVICE_H_
#define DEVICE_H_

#include "packets.h"

typedef enum DeviceState {
	Disconnected, Connected, InSleep
} DeviceState;

typedef struct Device {
	uint16_t address;
	DeviceState state;
} Device;

typedef struct DeviceProperties {
	uint16_t deviceType;
	uint16_t data;
	uint8_t *items;
	uint8_t *values;	
	uint8_t sleep;
	uint16_t readWrite;
} DeviceProperties;

typedef struct Node {
	Device device;
	DeviceProperties props;
} Node;

#endif  // DEVICE_H_