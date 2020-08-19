/*
 * ADC_lib.c
 *
 * Created: 9.3.2017 16:24:34
 *  Author: Krajsa
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "makra.h"
#include "ADC.h"
void ADC_Init(uint8_t prescale,uint8_t uref) {
	ADMUX=0;
	ADCSRA=0;
	ADCSRA |= (prescale<<ADPS0);
	ADMUX |= (uref<<REFS0);
	//Kontrola jesli jsou napeti OK
	sbi(ADCSRA,ADEN);
	while(!(ADCSRB & 0x80));
	while(!(ADCSRB & 0x20));
}
void ADC_stop(void){
	cbi(ADCSRA,ADEN);
} 
uint16_t ADC_get(uint8_t chan) {
	uint16_t tmp=0;
	//smazat MUX
	ADMUX &= ~(31 << MUX0); // copak ten zapis znamena?
	ADCSRB&= ~(1 << MUX5);
	
	//nastavit spravny kanal
	ADMUX |= (chan<<MUX0);
	ADCSRA |= (1<<ADSC); // spustit konverzi
	while((tbi(ADCSRA,ADSC))){} //pockat nez skonci, sice polling, ale proc ne, je to rychle a nezavysle
	//pomocna = ADCL | (ADCH << 8);
	tmp=ADC; 
	ADCSRA |= (1<<ADIF); //vynulovat priznak konverze - tim ze zapiseme 1, viz datasheet
	return tmp;
}
uint16_t ADC_getOffset(uint8_t chan) {		//offset 1000
	uint16_t tmp=0;
	//smazat MUX
	ADMUX &= ~(31 << MUX0); // copak ten zapis znamena?
	ADCSRB&= ~(1 << MUX5);
	//nastavit spravny kanal
	ADMUX |= (chan<<MUX0);
	ADCSRA |= (1<<ADSC); // spustit konverzi
	while((tbi(ADCSRA,ADSC))){} //pockat nez skonci, sice polling, ale proc ne, je to rychle a nezavysle
	//pomocna = ADCL | (ADCH << 8);
	tmp=ADC;
	ADCSRA |= (1<<ADIF); //vynulovat priznak konverze - tim ze zapiseme 1, viz datasheet
	if (((tmp >> 9)  & 0x01) == 1)
	{
		tmp = 1000 - (~tmp);
	}
	else
	{
		tmp = 1000 + tmp;
	}
	return tmp;
}
uint16_t ADC_readTemp(){
	uint16_t tmp=0;
	ADMUX &= ~(15 << MUX0);
	ADCSRB&= ~(1 << MUX5);
	ADCSRB|= (1<<MUX5);
	ADMUX |= (0b01001<<MUX0);
	ADCSRA |= 0x40; // spustit konverzi
	while((tbi(ADCSRA,ADSC))){} //pockat nez skonci
	//pomocna = ADCL | (ADCH << 8);
	tmp=ADC;
	ADCSRA |= (1<<ADIF);
	return tmp;
	
}
void ADC_Start_per(uint8_t prescale,uint8_t uref,uint8_t chan,uint8_t source){
		ADMUX=0;
		ADCSRA=0;
		ADCSRA |= (prescale<<ADPS0);
		ADMUX |= (uref<<REFS0);
		//Kontrola jesli jsou napeti OK
		sbi(ADCSRA,ADEN);
		while(!(ADCSRB & 0x80));
		while(!(ADCSRB & 0x20));
		//nastavit kanal
		ADMUX |= (chan<<MUX0);
		ADCSRB |= (source<<ADTS0);
		ADCSRA |= (1<<ADATE);
		ADCSRA |= (1<<ADIE); //Povolit preruseni, prece to nebudeme porad kontrolovat pollingem
}