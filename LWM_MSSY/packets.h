/*
 * packets.h
 *
 * Created: 21.3.2019 10:01:25
 *  Author: Student
 */ 


#ifndef PACKETS_H_
#define PACKETS_H_

#include <stdint.h>

typedef enum PacketType {
	HelloPacket = 1, HelloAckPacket, AckPacket, SleepPacket,
	ReconnectPacket, DataPacket, SetValuePacket, GetValuePacket, UnknownPacket
} PacketType;

typedef struct GetValuePacket_t {
	uint8_t command_id;
	uint8_t device_type;
	uint16_t data;
	uint8_t* items;
} GetValuePacket_t;

typedef struct HelloPacket_t {
	GetValuePacket_t data_part;
	uint8_t sleep;
	uint16_t read_write;
} HelloPacket_t;

typedef struct HelloAckPacket_t {
	uint8_t command_id;
	uint16_t address;
	uint16_t sleep_period;
	uint16_t reserved;
} HelloAckPacket_t;

typedef struct SleepPacket_t {
	uint8_t command_id;
	uint16_t sleep_period;
	uint16_t reserved;
} SleepPacket_t;

typedef struct ReconnectAckPacket_t {
	uint8_t command_id;
	uint8_t data;
	uint16_t reserved;
} ReconnectAckPacket_t;

typedef struct DataPacket_t {
	uint8_t device_type;
	uint16_t data;
	uint8_t* items;
	uint8_t* values;
} DataPacket_t;

typedef struct SetValuePacket_t {
	uint8_t command_id;
	DataPacket_t data_part;
} SetValuePacket_t;

#endif /* PACKETS_H_ */