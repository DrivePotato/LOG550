/*
 * setup.h
 *
 * Created: 2018-02-18 09:08:00
 *  Author: komlan
 */ 


#ifndef SETUP_H_
#define SETUP_H_


#define   TRUE   1
#define   FALSE   0

/*   Timer counter constant */
#  define TC_CHANNEL_0					0
#  define TC_CHANNEL_1					1


#  define TC							(&AVR32_TC)
#  define TC_IRQ_GROUP					AVR32_TC_IRQ_GROUP
#  define TC_IRQ0						AVR32_TC_IRQ0
#  define TC_IRQ1						AVR32_TC_IRQ1

#  define FPBA							FOSC0          // FOSC0 est a 12Mhz


#define FRQ0							FOSC0 / 32//Horloge 0
#define FRQ1							FOSC0 / 2//Horloge 1



// ADC
# define ADC_POT_MASK					0b11111110
# define ADC_LIGHT_MASK					0x01
# define ADC_BASE_FRQ					8000 //8kHz 1000 ech/sec
# define ADC_DBL_FRQ					16000 //16kHz 2000 ech/sec

//USART
# define BAUDS_RATE						57600
#endif /* SETUP_H_ */