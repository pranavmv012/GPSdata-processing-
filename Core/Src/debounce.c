/*
 * debounce.txt
 *
 *  Created on: April 05, 2020
 *      Author: Pranav Manakkulamparambil Vasudevan
 */

#include "debounce.h"
// Defining functions

void debounceInit()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
//configuring GPIOs as output push pull
	GPIO_InitStruct.Pin = GPIO_PIN_0;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
	GPIO_InitStruct.Pull = GPIO_PULLUP;//initializing the pull of GPIO_initstruct
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//Initializing GPIOA with the GPIO_InitStruct
}
void debounceInit2()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
//configuring GPIOs as output push pull
	GPIO_InitStruct.Pin = GPIO_PIN_0;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
	GPIO_InitStruct.Pull = GPIO_PULLUP;//initializing the pull of GPIO_initstruct
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//Initializing GPIOA with the GPIO_InitStruct
}
void debounceInit3()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
//configuring GPIOs as output push pull
	GPIO_InitStruct.Pin = GPIO_PIN_0;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
	GPIO_InitStruct.Pull = GPIO_PULLUP;//initializing the pull of GPIO_initstruct
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//Initializing GPIOA with the GPIO_InitStruct
}
void debounceInit4()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
//configuring GPIOs as output push pull
	GPIO_InitStruct.Pin = GPIO_PIN_0;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
	GPIO_InitStruct.Pull = GPIO_PULLUP;//initializing the pull of GPIO_initstruct
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//Initializing GPIOA with the GPIO_InitStruct
}

void deBounceInit(uint16_t pin, char port, int8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0}; // Declaring GPIO_initStruct as GPIO_InitTypeDef Struct
    GPIO_InitStruct.Pin= pin;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
    switch(mode)//select the alternate pull for the input port
    {
    case 0 : GPIO_InitStruct.Pull = GPIO_PULLUP; break;
    case 1 : GPIO_InitStruct.Pull = GPIO_PULLDOWN;break;
    }
    switch(port)//select which port to read from
    {
    case 'A':HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); break;
    case 'B':HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
    case 'C':HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);break;
    }

}

void deBounceInit2(uint16_t pin2, char port, int8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0}; // Declaring GPIO_initStruct as GPIO_InitTypeDef Struct
    GPIO_InitStruct.Pin= pin2;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
    switch(mode)//select the alternate pull for the input port
    {
    case 0 : GPIO_InitStruct.Pull = GPIO_PULLUP; break;
    case 1 : GPIO_InitStruct.Pull = GPIO_PULLDOWN;break;
    }
    switch(port)//select which port to read from
    {
    case 'A':HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); break;
    case 'B':HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
    case 'C':HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);break;
    }

}
void deBounceInit3(uint16_t pin3, char port, int8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0}; // Declaring GPIO_initStruct as GPIO_InitTypeDef Struct
    GPIO_InitStruct.Pin= pin3;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
    switch(mode)//select the alternate pull for the input port
    {
    case 0 : GPIO_InitStruct.Pull = GPIO_PULLUP; break;
    case 1 : GPIO_InitStruct.Pull = GPIO_PULLDOWN;break;
    }
    switch(port)//select which port to read from
    {
    case 'A':HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); break;
    case 'B':HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
    case 'C':HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);break;
    }

}
void deBounceInit4(uint16_t pin4, char port, int8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0}; // Declaring GPIO_initStruct as GPIO_InitTypeDef Struct
    GPIO_InitStruct.Pin= pin4;// initializing the pin of GPIO_initstruct
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//initializing the mode of GPIO_initstruct
    switch(mode)//select the alternate pull for the input port
    {
    case 0 : GPIO_InitStruct.Pull = GPIO_PULLUP; break;
    case 1 : GPIO_InitStruct.Pull = GPIO_PULLDOWN;break;
    }
    switch(port)//select which port to read from
    {
    case 'A':HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); break;
    case 'B':HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
    case 'C':HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);break;
    }

}
//function to read button 1
int8_t deBounceReadPin(uint16_t pin, char port, int8_t mode)
{
	GPIO_PinState pinState=GPIO_PIN_RESET;
	int8_t pinStateWeAreLookingFor;
	int8_t stableInterval=50;
	int32_t MsTimestamp= HAL_GetTick();

	switch(port)//select which port to read from
	    {
	    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin); break;
	    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin); break;
	    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin); break;
	    }
	if (pinState==GPIO_PIN_RESET)
	{
		pinStateWeAreLookingFor=0;
	}
	else
	{
		pinStateWeAreLookingFor=1;
	}

	while(HAL_GetTick()<(MsTimestamp+stableInterval))
	{
		switch(port)//select which port to read from
			    {
			    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin); break;
			    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin); break;
			    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin); break;
			    }
		if (pinState!=pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor= !pinStateWeAreLookingFor;
			MsTimestamp-HAL_GetTick();
		}

	}
	return (pinStateWeAreLookingFor);

}
//function to read button 2

int8_t deBounceReadPin2(uint16_t pin2, char port, int8_t mode)
{
	GPIO_PinState pinState=GPIO_PIN_RESET;
	int8_t pinStateWeAreLookingFor;
	int8_t stableInterval=50;
	int32_t MsTimestamp= HAL_GetTick();

	switch(port)//select which port to read from
	    {
	    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin2); break;
	    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin2); break;
	    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin2); break;
	    }
	if (pinState==GPIO_PIN_RESET)
	{
		pinStateWeAreLookingFor=0;
	}
	else
	{
		pinStateWeAreLookingFor=1;
	}

	while(HAL_GetTick()<(MsTimestamp+stableInterval))
	{
		switch(port)//select which port to read from
			    {
			    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin2); break;
			    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin2); break;
			    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin2); break;
			    }
		if (pinState!=pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor= !pinStateWeAreLookingFor;
			MsTimestamp-HAL_GetTick();
		}

	}
	return (pinStateWeAreLookingFor);

}
//function to read button 3

int8_t deBounceReadPin3(uint16_t pin3, char port, int8_t mode)
{
	GPIO_PinState pinState=GPIO_PIN_RESET;
	int8_t pinStateWeAreLookingFor;
	int8_t stableInterval=50;
	int32_t MsTimestamp= HAL_GetTick();

	switch(port)//select which port to read from
	    {
	    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin3); break;
	    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin3); break;
	    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin3); break;
	    }
	if (pinState==GPIO_PIN_RESET)
	{
		pinStateWeAreLookingFor=0;
	}
	else
	{
		pinStateWeAreLookingFor=1;
	}

	while(HAL_GetTick()<(MsTimestamp+stableInterval))
	{
		switch(port)//select which port to read from
			    {
			    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin3); break;
			    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin3); break;
			    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin3); break;
			    }
		if (pinState!=pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor= !pinStateWeAreLookingFor;
			MsTimestamp-HAL_GetTick();
		}

	}
	return (pinStateWeAreLookingFor);

}
//function to read button ok

int8_t deBounceReadPin4(uint16_t pin4, char port, int8_t mode)
{
	GPIO_PinState pinState=GPIO_PIN_RESET;
	int8_t pinStateWeAreLookingFor;
	int8_t stableInterval=50;
	int32_t MsTimestamp= HAL_GetTick();

	switch(port)//select which port to read from
	    {
	    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin4); break;
	    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin4); break;
	    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin4); break;
	    }
	if (pinState==GPIO_PIN_RESET)
	{
		pinStateWeAreLookingFor=0;
	}
	else
	{
		pinStateWeAreLookingFor=1;
	}

	while(HAL_GetTick()<(MsTimestamp+stableInterval))
	{
		switch(port)//select which port to read from
			    {
			    case 'A':pinState= HAL_GPIO_ReadPin(GPIOA, pin4); break;
			    case 'B':pinState= HAL_GPIO_ReadPin(GPIOB, pin4); break;
			    case 'C':pinState= HAL_GPIO_ReadPin(GPIOC, pin4); break;
			    }
		if (pinState!=pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor= !pinStateWeAreLookingFor;
			MsTimestamp-HAL_GetTick();
		}

	}
	return (pinStateWeAreLookingFor);

}

