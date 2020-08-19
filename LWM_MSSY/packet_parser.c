/*
 * packet_parser.c
 *
 * Created: 22. 3. 2019 21:38:04
 *  Author: Maros
 */ 

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

#include "device.h"
#include "packet_parser.h"
#include "packets.h"
#include "halUart.h"

//#define DEBUGGING

PacketType get_packet_type(Device *device, NWK_DataInd_t *nwk_packet) {
	uint8_t first_byte = nwk_packet->data[0];
	switch (nwk_packet->dstEndpoint) {
		case 1: {
			if (device->state == Disconnected && first_byte == 128) {  // node -> gateway
				return HelloPacket;
			} 
			else if(device->state == Disconnected && first_byte == 2) { // gateway -> node
				return HelloAckPacket;
			}
			else if (device->state == Connected && first_byte == 64) {  // node -> gateway
				return SleepPacket;
			}
			else if (device->state == InSleep && first_byte == 32) {  // node -> gateway
				return ReconnectPacket;
			} 
			else {
				return AckPacket;
			}
		} break;
		
		case 2: {
			if (device->state == Connected && first_byte == 128) {  // gateway -> node
				return SetValuePacket;
			} else if (device->state == Connected && first_byte == 64) {  // gateway -> node
				return GetValuePacket;
			} else if (first_byte == 1) { // both
				return AckPacket;
			} else {   // node -> gateway
				return DataPacket;
			}
		} break;
		
		case 3: {
			if (nwk_packet->size >= 6 && first_byte == 2) {  // node -> gateway
				return DataPacket;
			}
			else { //if (frame[0] == 1) { // both
				return AckPacket;
			}
		} break;
		
		case 4: {
			// ALARM NOT IMPLEMENTED YET; NOT SPECIFIED	
		}
		
		default:
			return UnknownPacket;
	}
}

PacketType process_packet(Device *device, NWK_DataInd_t *nwk_packet, void *packet_struct) {
	PacketType detected_packet = get_packet_type(device, nwk_packet);
	switch (detected_packet)
	{
		case HelloPacket: {
			packet_struct = (HelloPacket_t *)malloc(sizeof(HelloPacket_t));
			HelloPacket_t *packet = (HelloPacket_t *)packet_struct;
			packet->data_part.items = (uint8_t *)malloc((nwk_packet->size - 7));
			memcpy(packet, nwk_packet->data, 4);
			memcpy(&packet->sleep, &nwk_packet->data[5], 3);
			memcpy(packet->data_part.items, &nwk_packet->data[4], nwk_packet->size - 7);		
#ifdef DEBUGGING
			APP_WriteString("Hello packet - COMMAND: ");
			HAL_UartWriteByte(packet->data_part.command_id - 64);
			APP_WriteString(", DEVICE TYPE: ");
			HAL_UartWriteByte(packet->data_part.device_type + 20);
			APP_WriteString(", DATA: ");
			HAL_UartWriteByte(packet->data_part.data + 32);
			HAL_UartWriteByte(',');
			HAL_UartWriteByte((packet->data_part.data>>8) + '0');
			APP_WriteString(", ITEMS: ");
			HAL_UartWriteByte(packet->data_part.items[0] + '0');
			APP_WriteString(", SLEEP: ");
			HAL_UartWriteByte(packet->sleep - 200);
			APP_WriteString(", READ/WRITE: ");
			HAL_UartWriteByte(packet->read_write + '0');
			APP_WriteString("\r\n");
#endif // DEBUGGING
			break;
		}

		case HelloAckPacket: {
			packet_struct = (HelloAckPacket_t *) malloc(sizeof(HelloAckPacket_t));
			HelloAckPacket_t *packet = (HelloAckPacket_t *)packet_struct;
			memcpy(packet, nwk_packet->data, nwk_packet->size);
#ifdef DEBUGGING			
			APP_WriteString("Hello ACK packet - COMMAND: ");
			HAL_UartWriteByte(packet->command_id + 50);
			APP_WriteString(", ADDRESS: ");
			HAL_UartWriteByte(packet->address + 50);
			HAL_UartWriteByte(',');
			HAL_UartWriteByte((packet->address>>8) + 50);
			APP_WriteString(", SLEEP_PERIOD: ");
			HAL_UartWriteByte(packet->sleep_period - 200);
			HAL_UartWriteByte(',');
			HAL_UartWriteByte((packet->sleep_period>>8) - 200);
			APP_WriteString(", RESERVED: ");
			HAL_UartWriteByte(packet->reserved + '0');
			HAL_UartWriteByte(',');
			HAL_UartWriteByte((packet->reserved>>8) + '0');
			APP_WriteString("\r\n");
#endif // DEBUGGING
			//process hello-ACK packet
			break;
		}

		case AckPacket: {
			switch (nwk_packet->size)
			{
				case 7: {
					packet_struct = (HelloAckPacket_t *) malloc(sizeof(HelloAckPacket_t));
					HelloAckPacket_t *packet = (HelloAckPacket_t *)packet_struct;
					memcpy(packet, nwk_packet->data, nwk_packet->size);				
#ifdef DEBUGGING
					printf("Hello ACK Packet - COMMAND: %d, ADDRESS: %d, SLEEP_PERIOD: %d, RESERVED: %d\r\n", packet->command_id,
					packet->address, packet->sleep_period, packet->reserved);
								
					APP_WriteString("Hello ACK packet - COMMAND: ");
					HAL_UartWriteByte(packet->command_id + 50);
					APP_WriteString(", ADDRESS: ");
					HAL_UartWriteByte(packet->address + 50);
					HAL_UartWriteByte(',');
					HAL_UartWriteByte((packet->address>>8) + 50);
					APP_WriteString(", SLEEP_PERIOD: ");
					HAL_UartWriteByte(packet->sleep_period - 200);
					HAL_UartWriteByte(',');
					HAL_UartWriteByte((packet->sleep_period>>8) - 200);
					APP_WriteString(", RESERVED: ");
					HAL_UartWriteByte(packet->reserved + '0');
					HAL_UartWriteByte(',');
					HAL_UartWriteByte((packet->reserved>>8) + '0');
					APP_WriteString("\r\n");
#endif // DEBUGGING
					//process ACK-to-hello-ACK packet
					break;
				}
				case 5: {
					packet_struct = (SleepPacket_t *) malloc(sizeof(SleepPacket_t));
					SleepPacket_t *packet = (SleepPacket_t *)packet_struct;
					memcpy(packet, nwk_packet->data, nwk_packet->size);
#ifdef DEBUGGING
					APP_WriteString("Sleep packet - COMMAND: ");
					HAL_UartWriteByte(packet->command_id);
					APP_WriteString(", SLEEP_PERIOD: ");
					HAL_UartWriteByte(packet->sleep_period);
					HAL_UartWriteByte(',');
					HAL_UartWriteByte(packet->sleep_period>>8);
					APP_WriteString(", RESERVED: ");
					HAL_UartWriteByte(packet->reserved);
					HAL_UartWriteByte(',');
					HAL_UartWriteByte(packet->reserved>>8);
					APP_WriteString("\r\n");
#endif // DEBUGGING
					// process sleep-ACK packet
					break;
				}
				default: {
					packet_struct = (ReconnectAckPacket_t *) malloc(sizeof(ReconnectAckPacket_t));
					ReconnectAckPacket_t *packet = (ReconnectAckPacket_t *)packet_struct;
					memcpy(packet, nwk_packet->data, nwk_packet->size);
#ifdef DEBUGGING
					APP_WriteString("Reconnect packet - COMMAND: ");
					HAL_UartWriteByte(packet->command_id);
					APP_WriteString(", DATA: ");
					HAL_UartWriteByte(packet->data);
					APP_WriteString(", RESERVED: ");
					HAL_UartWriteByte(packet->reserved);
					HAL_UartWriteByte(',');
					HAL_UartWriteByte(packet->reserved>>8);
					APP_WriteString("\r\n");
#endif // DEBUGGING
					// process reconnect-ACK or data-ACK packet
					break;
				}
			}
			break;
		}

		case SleepPacket: {
			packet_struct = (SleepPacket_t *) malloc(sizeof(SleepPacket_t));
			SleepPacket_t *packet = (SleepPacket_t *)packet_struct;
			memcpy(packet, nwk_packet->data, nwk_packet->size);
#ifdef DEBUGGING
			APP_WriteString("Sleep packet - COMMAND: ");
			HAL_UartWriteByte(packet->command_id);
			APP_WriteString(", SLEEP_PERIOD: ");
			HAL_UartWriteByte(packet->sleep_period);
			HAL_UartWriteByte(',');
			HAL_UartWriteByte(packet->sleep_period>>8);
			APP_WriteString(", RESERVED: ");
			HAL_UartWriteByte(packet->reserved);
			HAL_UartWriteByte(',');
			HAL_UartWriteByte(packet->reserved>>8);
			APP_WriteString("\r\n");
#endif // DEBUGGING
			// process sleep packet
			break;
		}

		case ReconnectPacket: {
			packet_struct = (ReconnectAckPacket_t *) malloc(sizeof(ReconnectAckPacket_t));
			ReconnectAckPacket_t *packet = (ReconnectAckPacket_t *)packet_struct;
			memcpy(packet, nwk_packet->data, nwk_packet->size);
#ifdef DEBUGGING
			APP_WriteString("Reconnect packet - COMMAND: ");
			HAL_UartWriteByte(packet->command_id);
			APP_WriteString(", DATA: ");
			HAL_UartWriteByte(packet->data);
			APP_WriteString(", RESERVED: ");
			HAL_UartWriteByte(packet->reserved);
			HAL_UartWriteByte(',');
			HAL_UartWriteByte(packet->reserved>>8);
			APP_WriteString("\r\n");
#endif // DEBUGGING
			// process reconnect packet
			break;
		}

		case DataPacket: {
			packet_struct = (DataPacket_t *) malloc(sizeof(DataPacket_t));
			DataPacket_t *packet = (DataPacket_t *)packet_struct;
			memcpy(&packet->device_type, nwk_packet->data, 3);
			
			uint8_t item_count = 0;
			detect_data_packet_arrays_size(packet->data, &item_count);
			uint8_t data_bytes = nwk_packet->size - item_count - 3;
			
			packet->items = (uint8_t *) malloc(item_count);
			packet->values = (uint8_t *) malloc(data_bytes);
			memcpy(packet->items, &nwk_packet->data[3], item_count);
			memcpy(packet->values, &nwk_packet->data[3 + item_count], data_bytes);

#ifdef DEBUGGING
			APP_WriteString("Set value packet - DEVICE_TYPE: ");
			HAL_UartWriteByte(packet->device_type);
			APP_WriteString(", DATA: ");
			HAL_UartWriteByte(packet->data);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data>>8);
			APP_WriteString("\r\nITEMS: ");
			HAL_UartWriteByte(packet->items[0]+'0');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->items[1]+'1');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->items[2]+'2');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->items[3]+'3');
			APP_WriteString(", VALUES: ");
			HAL_UartWriteByte(packet->values[0]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[1]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[2]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[3]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[4]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[5]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[6]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[7]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[8]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->values[9]);
			APP_WriteString("\r\n");

#endif // DEBUGGING
			// process data packet
			break;
		}

		case GetValuePacket: {
			packet_struct = (GetValuePacket_t *) malloc(sizeof(GetValuePacket_t));
			GetValuePacket_t *packet = (GetValuePacket_t *)packet_struct;
			packet->items = (uint8_t *) malloc((nwk_packet->size - 4));
			memcpy(packet, nwk_packet->data, 4);
			memcpy(packet->items, &nwk_packet->data[4], nwk_packet->size - 4);
			
#ifdef DEBUGGING
			APP_WriteString("Set value packet - COMMAND: ");
			HAL_UartWriteByte(packet->command_id);
			APP_WriteString(", DEVICE_TYPE: ");
			HAL_UartWriteByte(packet->device_type);
			APP_WriteString(", DATA: ");
			HAL_UartWriteByte(packet->data);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data>>8);
			APP_WriteString(", ITEMS: ");
			HAL_UartWriteByte(packet->items[0]+'0');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->items[1]+'1');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->items[2]+'2');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->items[3]+'3');
			APP_WriteString("\r\n");
#endif // DEBUGGING
			// process get value packet
			break;
		}

		case SetValuePacket: {
			packet_struct = (SetValuePacket_t *) malloc(sizeof(SetValuePacket_t));
			SetValuePacket_t *packet = (SetValuePacket_t *)packet_struct;
			packet->command_id = nwk_packet->data[0];
			memcpy(&packet->data_part.device_type, &nwk_packet->data[1], 3);
			
			uint8_t item_count = 0;
			detect_data_packet_arrays_size(packet->data_part.data, &item_count);
			uint8_t data_bytes = nwk_packet->size - item_count - 4;

			packet->data_part.items = (uint8_t *) malloc(item_count);
			packet->data_part.values = (uint8_t *) malloc(data_bytes);
			memcpy(packet->data_part.items, &nwk_packet->data[4], item_count);
			memcpy(packet->data_part.values, &nwk_packet->data[4 + item_count], data_bytes);
			
#ifdef DEBUGGING
			APP_WriteString("Set value packet - COMMAND: ");
			HAL_UartWriteByte(packet->command_id);
			APP_WriteString(", DEVICE_TYPE: ");
			HAL_UartWriteByte(packet->data_part.device_type);
			APP_WriteString(", DATA: ");
			HAL_UartWriteByte(packet->data_part.data);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.data>>8);
			APP_WriteString("\r\nITEMS: ");
			HAL_UartWriteByte(packet->data_part.items[0]+'0');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.items[1]+'1');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.items[2]+'2');
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.items[3]+'3');
			APP_WriteString(", VALUES: ");
			HAL_UartWriteByte(packet->data_part.values[0]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[1]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[2]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[3]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[4]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[5]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[6]);
			APP_WriteString(", ");
			HAL_UartWriteByte(packet->data_part.values[7]);
			APP_WriteString("\r\n");
#endif // DEBUGGING
			//process set value packet
			break;
		}
		
		case UnknownPacket: {
#ifdef DEBUGGING
			APP_WriteString("Unknown packet received! \r\n");
#endif // DEBUGGING
			break;	
		}
	}
	return detected_packet;
}

void detect_data_packet_arrays_size(uint16_t data, uint8_t *item_count) {
	uint16_t temp;
	for (int i = 15; i >= 0; i--) {
		temp = (1 << i);
		if (data >= temp) {
			++(*item_count);
			data -= temp;
		}
	}
}

void get_values_bytesize(uint16_t data, uint8_t *count) {
	uint16_t temp_data = data, temp_value;
	for (int i = 15; i >= 0; i--) {
		temp_value = (1 << i);
		if (temp_data >= temp_value) {
			temp_data -= temp_value;
			switch (i) {
				case 1: case 2: case 3: case 15: {*(count) += 4; break; }
				case 4: case 5: case 6: case 13: {*(count) += 1; break; }
				case 7: case 14: {*(count) += 2; break;}
			}
		}
		if (temp_data == 0) break;
	}
}

void debug_packet(uint8_t *packet, uint8_t packet_length) {
	for (uint8_t i = 0; i < packet_length; ++i) {
		HAL_UartWriteByte(packet[i] + 50);
		HAL_UartWriteByte(',');
		HAL_UartWriteByte(' ');
	}
}

void serialize_fixed_size_packet(void* packet, uint8_t *frame_payload, uint8_t packet_length) {
	memcpy(frame_payload, packet, packet_length);
#ifdef DEBUGGING
	debug_packet(frame_payload, packet_length);
#endif // DEBUGGING
}

void serialize_hello_packet(HelloPacket_t *hello_packet, uint8_t *frame_payload, uint8_t packet_length) {
	memcpy(frame_payload, hello_packet, 4);
	memcpy(&frame_payload[packet_length - 4], hello_packet->data_part.items, packet_length - 7);
	memcpy(&frame_payload[packet_length - 3], &hello_packet->sleep, 3);
#ifdef DEBUGGING
	debug_packet(frame_payload, packet_length);
#endif // DEBUGGING
}

void serialize_hello_ack_packet(HelloAckPacket_t *hello_ack_packet, uint8_t *frame_payload) {
	serialize_fixed_size_packet(hello_ack_packet, frame_payload, 7);
}

void serialize_sleep_packet(SleepPacket_t * sleep_packet, uint8_t *frame_payload) {
	serialize_fixed_size_packet(sleep_packet, frame_payload, 5);
}

void serialize_reconnect_packet(ReconnectAckPacket_t *reconnect_packet, uint8_t *frame_payload) {
	serialize_fixed_size_packet(reconnect_packet, frame_payload, 4);
}

void serialize_get_value_packet(GetValuePacket_t *get_value_packet, uint8_t *frame_payload, uint8_t packet_length) {
	memcpy(frame_payload, get_value_packet, 4);
	memcpy(&frame_payload[4], get_value_packet->items, packet_length - 4);
#ifdef DEBUGGING
	debug_packet(frame_payload, packet_length);
#endif // DEBUGGING
}

void serialize_set_value_packet(SetValuePacket_t *set_value_packet, uint8_t *frame_payload, uint8_t packet_length, uint8_t item_count) {
	memcpy(frame_payload, set_value_packet, 4);
	memcpy(&frame_payload[4], set_value_packet->data_part.items, item_count);
	memcpy(&frame_payload[4 + item_count], set_value_packet->data_part.values, packet_length - item_count - 4);
#ifdef DEBUGGING
	debug_packet(frame_payload, packet_length);
#endif // DEBUGGING
}

void serialize_data_packet(DataPacket_t *data_packet, uint8_t *frame_payload, uint8_t packet_length, uint8_t item_count) {
	memcpy(frame_payload, data_packet, 3);
	memcpy(&frame_payload[3], data_packet->items, item_count);
	memcpy(&frame_payload[3 + item_count], data_packet->values, packet_length - item_count - 3);
#ifdef DEBUGGING
	debug_packet(frame_payload, packet_length);
#endif // DEBUGGING
}
