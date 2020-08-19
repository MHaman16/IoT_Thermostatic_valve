/*
 * LWM_MSSY.c
 *
 * Created: 6.4.2017 15:42:46
 * Author : Krajsa
 */ 

#include <avr/io.h>
#include <util/delay.h>
/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#include "halBoard.h"
#include "halUart.h"
#include "halTimer.h"
#include "main.h"
#include "ADC.h"
#include "TermostaicValve.h"
#include "i2cmaster.h"
#include "bmp085.h"
#include "device.h"
#include "packets.h"
#include "packet_parser.h"

/*- Definitions ------------------------------------------------------------*/
#define  Hystereze 1

void *packet_buffer;
PacketType last_packet_type;
Device node;

#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t
{
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;

volatile float setTemperature = 5;
volatile uint8_t TermostaticValveMod = 1; //0-manualni - tlacitka, 1 - hlidani teploty, 2 - nastaveni otevreni
volatile uint8_t connected = 0;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
appDataReqBusy = false;
if (req->status == NWK_SUCCESS_STATUS) {
(void)req;
APP_WriteString("[INFO]  Send complete!\r\n");
} else {
APP_WriteString("[ERROR] Send error!\r\n");
appSendData();
}
}

/*************************************************************************//**
*****************************************************************************/
static void prepareSendData(uint8_t destAddr, uint8_t endpoint, uint8_t* data, uint8_t length) {
memcpy(appDataReqBuffer, data, length);

appDataReq.dstAddr = destAddr;
appDataReq.dstEndpoint = endpoint;
appDataReq.srcEndpoint = endpoint;
appDataReq.options = NWK_OPT_ENABLE_SECURITY;
appDataReq.data = appDataReqBuffer;
appDataReq.size = appUartBufferPtr = length;
appDataReq.confirm = appDataConf;

//SYS_TimerStop(&appTimer);
//SYS_TimerStart(&appTimer);
//_delay_us(100);
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void) {
if (appDataReqBusy || 0 == appUartBufferPtr) {
APP_WriteString("[ERROR] Busy - cannot send data! \r\n");
_delay_us(100);
appSendData();
return;
}

NWK_DataReq(&appDataReq);

appDataReqBusy = true;
appUartBufferPtr = 0;
}

/*************************************************************************//**
*****************************************************************************/
void APP_sendHello() { //dev&testing only

node.address = 0x00;
node.state = Connected;
	

HelloPacket_t *packet = (HelloPacket_t *)malloc(sizeof(HelloPacket_t));
packet->data_part.command_id = 128;
packet->data_part.data = 32;
packet->data_part.device_type = 16;
packet->data_part.items = (uint8_t *)malloc(2);
packet->data_part.items[0] = 1;
packet->data_part.items[1] = 1;
packet->read_write = 66;
packet->sleep = 0;

uint8_t *frame_payload = (uint8_t *)malloc(9);
serialize_hello_packet(packet, frame_payload, 9);
prepareSendData(0x00, 1, frame_payload, 9);
appSendData();

APP_WriteString("\r\n[INFO]  Hello packet sent!\r\n");
}
/*************************************************************************//**
*****************************************************************************/
static void createSendHelloAck(NWK_DataInd_t *ind, uint16_t sleep_time, uint8_t command_id) {
APP_WriteString("[DEBUG] Sending ACK command: ");
HAL_UartWriteByte(command_id + '0'); //dev&testing only
APP_WriteString("\r\n");

HelloAckPacket_t *packet = (HelloAckPacket_t *)malloc(sizeof(HelloAckPacket_t));
packet->address = ind->srcAddr;
packet->command_id = command_id;
packet->reserved = 0;
packet->sleep_period = sleep_time;
uint8_t *frame_payload = (uint8_t *)malloc(7);
serialize_hello_ack_packet(packet, frame_payload);
prepareSendData(packet->address, 1, frame_payload, 7);
appSendData();
}

/*************************************************************************//**
*****************************************************************************/
static void createSendAck(NWK_DataInd_t *ind) {
prepareSendData(ind->srcAddr, ind->srcEndpoint, ind->data, ind->size); // check size value!!
appSendData();
}

static void createSendDataAck(NWK_DataInd_t *ind, uint8_t new_data) {
ReconnectAckPacket_t *packet = (ReconnectAckPacket_t *)malloc(sizeof(ReconnectAckPacket_t));
packet->command_id = 1;
packet->data = new_data;
packet->reserved = 0;
uint8_t *frame_payload = (uint8_t *)malloc(4);
serialize_reconnect_packet(packet, frame_payload);
prepareSendData(ind->srcAddr, ind->srcEndpoint, frame_payload, 4);
appSendData();
}


/*************************************************************************//**
*****************************************************************************/
void HAL_UartBytesReceived(uint16_t bytes)
{
for (uint16_t i = 0; i < bytes; i++)
{
uint8_t byte = HAL_UartReadByte();

if (appUartBufferPtr == sizeof(appUartBuffer))
appSendData();

if (appUartBufferPtr < sizeof(appUartBuffer))
appUartBuffer[appUartBufferPtr++] = byte;
}

SYS_TimerStop(&appTimer);
SYS_TimerStart(&appTimer);
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
appSendData();
(void)timer;
}
/*************************************************************************//**
*****************************************************************************/
uint8_t hex_char_to_value(uint8_t hex_char) {
uint8_t result = -1;
if (hex_char >= '0' && hex_char <= '9')
result = hex_char - '0';
else if (hex_char >= 'a' && hex_char <= 'f')
result = hex_char - 'a' + 10;
else if (hex_char >= 'A' && hex_char <= 'F')
result = hex_char - 'A' + 10;
return result;
}

void APP_hex_to_byte(uint8_t hex[], uint8_t byte[], uint8_t byte_length) {
for (uint8_t i = 0; i < byte_length; ++i) {
byte[i] = (hex_char_to_value(hex[2 * i]) << 4) + hex_char_to_value(hex[2 * i + 1]);
}
}

/*************************************************************************//**
*****************************************************************************/
uint8_t byte_to_hex_value(uint8_t char_value) {
return (char_value < 10) ? char_value + '0' : char_value + 'a' - 10;
}

void APP_byte_to_hex(uint8_t *byte, uint8_t *hex, uint8_t hex_length) {
for (uint8_t i = 0; i < hex_length; i+=2) {
hex[i] = byte_to_hex_value(byte[i / 2] >> 4);
hex[i+1] = byte_to_hex_value(byte[i / 2] & 0xf);
}
}
/*************************************************************************//**
*****************************************************************************/
static void printData(DataPacket_t *dataPacket, uint8_t packetLength) {
uint8_t item_count;
detect_data_packet_arrays_size(dataPacket->data, &item_count);
uint8_t values_size = packetLength - item_count - 3;
uint8_t *hex_values = (uint8_t *)malloc(values_size);
APP_byte_to_hex(dataPacket->values, hex_values, values_size);
APP_WriteString("[DEBUG] Data packet:");
for (uint8_t i = 0; i < values_size; ++i ) {
HAL_UartWriteByte(hex_values[i]);
HAL_UartWriteByte(' ');
}
APP_WriteString("\r\n---End Of Data---\r\n");
}
/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
createSendDataAck(ind, 0);
last_packet_type = process_packet(&node, ind, packet_buffer);
if (ind->dstEndpoint == 2 && last_packet_type == HelloAckPacket)
{
	connected == 1;
}

if (ind->dstEndpoint == 2 && last_packet_type == SetValuePacket)
{
	printData((DataPacket_t *)packet_buffer, ind->size); 
	packet_buffer = (DataPacket_t *)packet_buffer;
	if (packet_buffer->data == 2)	//teplota
	{
		TermostaticValveMod = 1;
		setTemperature = 0;
		setTemperature = setTemperature | values[0];
		setTemperature = (setTemperature << 8) | values[1];
		setTemperature = (setTemperature << 8) | values[2];
		setTemperature = (setTemperature << 8) | values[3];
	}
	else if (packet_buffer->data == 64)	//procenta
	{
		TermostaticValveMod = 2;
		setPosition(packet_buffer->values[0]);
	}
}
for (uint8_t i = 0; i < ind->size; i++)
HAL_UartWriteByte(ind->data[i]);
return true;
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
NWK_SetAddr(APP_ADDR);
NWK_SetPanId(APP_PANID);
PHY_SetChannel(APP_CHANNEL);
#ifdef PHY_AT86RF212
PHY_SetBand(APP_BAND);
PHY_SetModulation(APP_MODULATION);
#endif
PHY_SetRxState(true);

NWK_OpenEndpoint(NETWORK_ENDPOINT, appDataInd);
NWK_OpenEndpoint(GET_SET_ENDPOINT, appDataInd);

HAL_BoardInit();

appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
appTimer.mode = SYS_TIMER_INTERVAL_MODE;
appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
switch (appState)
{
case APP_STATE_INITIAL:
{
appInit();
appState = APP_STATE_IDLE;
} break;

case APP_STATE_IDLE:
break;

default:
break;
}
}
/*************************************************************************//**
*****************************************************************************/
void APP_WriteString(char *string) {
int index = 0;
while (string[index] != '\0') {
HAL_UartWriteByte(string[index]);
++index;
}
}
/*************************************************************************//**
*****************************************************************************/

void Thermostatic(){
	double d = bmp085_gettemperature();
	if ((d - Hystereze) < setTemperature)
	{
		openClose(1);
	}
	if ((d + Hystereze) > setTemperature)
	{
		openClose(0);
	}
}

void CheckButtons(){
	if (!tbi(PINE, PINE7))
	{
		openClose(0);
		TermostaticValveMod = 0;
		HAL_TimerDelay(200);
	}	
	if (!tbi(PINF, PINF3))
	{
		openClose(1);
		TermostaticValveMod = 0;
		HAL_TimerDelay(200);		
	}
}



int main(void)
{
	SYS_Init();
	HAL_UartInit(38400);
	initTermosticValve();
	i2c_init();
	bmp085_init();
	//Tlacitka vstup
	cbi(DDRG, DDG5);	//tlacitko 3
	cbi(DDRF, DDF3);	//tlacitko 2
	cbi(DDRE, DDE7);	//tlacitko 1
	
	//Tlacitka pull-up
	sbi(PORTG, 5);	//tlacitko 3
	sbi(PORTF, 3);	//tlacitko 2
	sbi(PORTE, 7);	//tlacitko 1
	HAL_UartWriteByte('a');
	uint16_t CycleCounter = 0;
	APP_sendHello();
	char vysledek_string[10];


	while (1)
	{
		SYS_TaskHandler();
		HAL_UartTaskHandler();
		APP_TaskHandler();
	
		CheckButtons();
	

		if (CycleCounter == 800)
		{
			if(TermostaticValveMod == 1) Thermostatic();
			if(connected == 0) APP_sendHello();
			CycleCounter = 0;
			
		}
	
	
	
		CycleCounter++;
	
		HAL_TimerDelay(2500);	
	}
}
