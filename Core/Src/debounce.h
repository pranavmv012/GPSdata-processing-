/*
 * debounce.h
 *
 *  Created on: Apr 10 ,2020
 *      Author: Pranav Manakkulamparambil Vasudevan
 */

#ifndef SRC_DEBOUNCE_H_
#define SRC_DEBOUNCE_H_
#include <stdint.h>
#include "stm32l4xx_hal.h"

// Declaring functions
//function for initializing the debouce
void debounceInit();
void debounceInit2();
void debounceInit3();
void debounceInit4();
//function for initializing pin, port and mode
void deBounceInit(uint16_t pin, char port, int8_t mode);
void deBounceInit2(uint16_t pin2, char port, int8_t mode);
void deBounceInit3(uint16_t pin3, char port, int8_t mode);
void deBounceInit4(uint16_t pin4, char port, int8_t mode);
// function to read pins
int8_t deBounceReadPin(uint16_t pin, char port, int8_t mode);
int8_t deBounceReadPin2(uint16_t pin2, char port, int8_t mode);
int8_t deBounceReadPin3(uint16_t pin3, char port, int8_t mode);
int8_t deBounceReadPin4(uint16_t pin4, char port, int8_t mode);


#endif /* SRC_DEBOUNCE_H_ */
