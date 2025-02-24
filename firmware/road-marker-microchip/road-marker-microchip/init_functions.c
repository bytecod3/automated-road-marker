/**
*@file: init_functions.c
*@author: Edwin Mwiti
*
*This file contains initializations functions for all the sub-systems used in this robot
*
* 
*/

 
#define F_CPU 16E6
#include "string.h"
#include "init_functions.h"

/** 
*@brief: initialize the uart peripheral on the chip
*/
void uart_init(void) {
	uint32_t ubrr = (F_CPU/16UL)/USART_BAUD - 1;
	UBRR0H = (unsigned char) (ubrr>>8);
	UBRR0L = ubrr;
	
	// enable receiver and transmitter
	UCSR0B |= (1<<RXEN0);
	UCSR0B |= (1<<TXEN0);
	
	// frame format to 8-bit, 2 stop-bits
	UCSR0C = (1<<USBS0) | (3 << UCSZ00);
}

/**
*@brief: Transmit a single character via uart 
*@param[in] c the character to transmit
*/
void uart_transmit(unsigned char c) {
	// wait for empty transmit buffer 
	while(!(UCSR0A &(1<< UDRE0)));
	UDR0 = c;
}

/**
*@brief: Transmit a string via uart
*@param[in]  s the string to transmit
*/
void uart_debug(char* s) {
	for (int i =0; i < strlen(s); i++) {
		uart_transmit(s[i]));
	}
}

/**
*@brief: initialize 
*/
void buzzer_init() {
	
}

/**
*@brief: initialize the ultrasonic sensing pins
*/
void ultrasonic_init() {
	
}
