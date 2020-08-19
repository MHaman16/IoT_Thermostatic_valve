/*
 * TermostaicValve.h
 *
 * Created: 28.3.2019 10:34:28
 *  Author: Student
 */ 
void initTermosticValve();
uint16_t ADC_Measure_Avg();
void DelayMs(uint16_t time);
void calibration();
void openClose(uint8_t directon);
void setPosition( uint8_t position);