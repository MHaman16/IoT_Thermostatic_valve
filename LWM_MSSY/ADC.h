/*
 * ADC_lib.h
 *
 * Created: 9.3.2017 16:24:50
 *  Author: Krajsa
 */ 


#ifndef ADC_LIB_H_
#define ADC_LIB_H_

void ADC_Init(uint8_t prescale,uint8_t uref);
void ADC_Start_per(uint8_t prescale,uint8_t uref,uint8_t chan,uint8_t source);
uint16_t ADC_get(uint8_t chan);
uint16_t ADC_getOffset(uint8_t chan);
uint16_t ADC_readTemp();
void ADC_stop(void);


#endif /* ADC_LIB_H_ */