/*
 * CFile1.c
 *
 * Created: 21.3.2019 10:09:57
 *  Author: Student
 */ 
#include "makra.h"
#include "ADC.h"
#include "halTimer.h"

//#define OpenPin
#define OpenTreshold 1300
#define CloseTreshold 1800
#define ClosePulseDurationMS 150
#define OpenPulseDurationMS 250
#define LimitToCalibration 50		//255 is maximum
#define DelayBeforeMeasure 800
#define NumberOfMeasures 200
#define MeasureDelay 80		//255 is maximum
#define DEBUG

#define OPENON sbi(PORTE,2)
#define OPENOFF cbi(PORTE,2)
#define CLOSEON sbi(PORTE,3)
#define CLOSEOFF cbi(PORTE,3)

void calibration();
void openClose(uint8_t directon);
void setPosition(uint8_t position);
void DelayMs(uint16_t time);
uint16_t ADC_Measure_Avg();


volatile uint8_t mod = 0;
volatile uint8_t position = 255;
volatile uint8_t CalibrationCounter = 0;
volatile uint16_t NumberOfPulsesClose = 0;
volatile uint16_t NumberOfPulsesOpen = 0;
char vysledek_string[10];
uint16_t tmp;

void initTermosticValve()
{
	HAL_UartWriteString("\r\n");
	HAL_UartWriteString("InitTermostaticValve \r\n");
	
	ADC_Init(4, 2);
	sbi(DDRE,2);	//output open
	sbi(DDRE,3);	//output close
	OPENOFF;
	CLOSEOFF;
	calibration();
}
uint16_t ADC_Measure_Avg(){
	float m = 0;
	for(uint8_t i=0; i < NumberOfMeasures; i++){
		m = m + ADC_getOffset(0b010000);
		HAL_TimerDelay(MeasureDelay);
	}
	return (uint16_t)(m/NumberOfMeasures);
}

void DelayMs(uint16_t time){
	for(time; time > 0; time--){
		HAL_TimerDelay(1000);
	}
}


void calibration()
{
	//#ifdef DEBUG
	HAL_UartWriteString("\r\n");
	HAL_UartWriteString("Calibration Start \r\n");
	//#endif
	uint8_t previousPosition = position;
	openClose(0);
	NumberOfPulsesOpen = 0;
	NumberOfPulsesClose = 0;
	CalibrationCounter = 0;
	OPENON;
	CLOSEOFF;
	DelayMs(DelayBeforeMeasure);
	uint16_t adcValue = ADC_Measure_Avg();
	while(OpenTreshold < adcValue)
	{
		NumberOfPulsesOpen ++;		
		DelayMs(OpenPulseDurationMS);
		adcValue = ADC_Measure_Avg();
	}
	OPENOFF;
	CLOSEOFF;
	//#ifdef DEBUG
	HAL_UartWriteString("\r\n");
	HAL_UartWriteString("Full open pulses: \r\n");
	itoa(NumberOfPulsesOpen, vysledek_string, 10);
	HAL_UartWriteString(vysledek_string);
	HAL_UartWriteString("\r\n");
	//#endif
	OPENOFF;
	CLOSEON;
	DelayMs(DelayBeforeMeasure);
	adcValue = ADC_Measure_Avg();
	while(CloseTreshold > adcValue)
	{
		NumberOfPulsesClose ++;
		
		DelayMs(ClosePulseDurationMS);
		adcValue = ADC_Measure_Avg();
	}
	OPENOFF;
	CLOSEOFF;
	position = 0;
	setPosition(previousPosition);
	
	//#ifdef DEBUG
	HAL_UartWriteString("\r\n");
	HAL_UartWriteString("Full close pulses: \r\n");
	itoa(NumberOfPulsesClose, vysledek_string, 10);
	HAL_UartWriteString(vysledek_string);
	HAL_UartWriteString("\r\n");
	HAL_UartWriteString("\r\n");
	HAL_UartWriteString("Calibration complete \r\n");
	//#endif
}

void openClose(uint8_t direction)	//1 - open, 0 - close
{
	if (direction == 0 && position != 0)		//close valve
	{
		CalibrationCounter ++;
		if(CalibrationCounter>LimitToCalibration) calibration();
		OPENOFF;
		CLOSEON;
		DelayMs(DelayBeforeMeasure);
		uint16_t adcValue = ADC_Measure_Avg();
		while(CloseTreshold > adcValue)
		{
			adcValue = ADC_Measure_Avg();
		}
		position = 0;
		
		//#ifdef DEBUG
		vysledek_string[0] = 0;
		HAL_UartWriteString("Valve is closed: \r\n");
		itoa(adcValue, vysledek_string, 10);
		HAL_UartWriteString(vysledek_string);
		HAL_UartWriteString("\r\n");
		//#endif
		
	}
	if (direction == 1 && position != 100)		//open valve
	{
		CalibrationCounter ++;
		if(CalibrationCounter>LimitToCalibration) calibration();
		
		OPENON;
		CLOSEOFF;
		DelayMs(DelayBeforeMeasure);		
		uint16_t adcValue = ADC_Measure_Avg();
		while(OpenTreshold < adcValue)
		{
			adcValue = ADC_Measure_Avg();
		}
		position = 100;
		
		//#ifdef DEBUG
		//tmp = ADC_Measure_Avg();
		vysledek_string[0] = 0;
		HAL_UartWriteString("Valve is opened: \r\n");
		itoa(adcValue, vysledek_string, 10);
		HAL_UartWriteString(vysledek_string);
		HAL_UartWriteString("\r\n");
		//#endif
	}
	OPENOFF;
	CLOSEOFF;
	return;
}

void setPosition(uint8_t positionToSet)	//position - position in % 0=close
{
	if(positionToSet>100) return;
	CalibrationCounter ++;
	if(CalibrationCounter>LimitToCalibration) calibration();
	if (position<positionToSet)			//valve opening
	{
		uint8_t steps = (uint8_t)((NumberOfPulsesOpen/100)*(positionToSet-position));
		OPENON;
		CLOSEOFF;
		for (steps; steps>0; steps --)
		{
			DelayMs(OpenPulseDurationMS);
		}
		OPENOFF;
		CLOSEOFF;
	} 
	else		//valve closing
	{
		uint8_t steps = (uint8_t)((NumberOfPulsesClose/100)*(position-positionToSet));
		OPENOFF;
		CLOSEON;
		for (steps; steps>0; steps --)
		{
			DelayMs(ClosePulseDurationMS);
		}
		OPENOFF;
		CLOSEOFF;
	}
	position = positionToSet;
}
