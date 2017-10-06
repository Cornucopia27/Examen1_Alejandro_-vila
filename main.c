/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Tarea_1_LED_RGB.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "DataTypeDefinitions.h"
#include "MK64F12.h"
#include "NVIC.h"
#include "GPIO.h"

/**Macros to declare RUN and STOP as state 1 and 0 of Motor*/
#define ON &LED[0]
#define OFF &LED[1]

/**Defines the struct of states, holds a flag to switch between states*/
typedef const struct State
{
	uint8 flag_sec;
	const struct State* next[2];
}StateType;

/**Struct created to make a statemachine to turn on and off the motor*/
const StateType LED[2]=
{
		{1, {OFF,ON}},/**On*/
		{0, {ON, OFF}}/**Off*/

};
//static StateType* State_LED = ON;

void delay(uint16 delay);
void turnLEDsOff();
void blueLEDOn();
void redLEDOn();
void greenLEDOn();
void yellowColor();
void purpleColor();
void whiteColor();

static void (*FunctionPoint[7])(void) = {greenLEDOn, blueLEDOn, purpleColor, redLEDOn, yellowColor, whiteColor, turnLEDsOff};
/*
 * @brief   Application entry point.
 */
int main(void)
{
	/**Activating the clock gating of the GPIOs*/
	GPIO_clockGating(GPIO_A);
	GPIO_clockGating(GPIO_B);
	GPIO_clockGating(GPIO_C);
	GPIO_clockGating(GPIO_D);
	/**Selected configurations SW2 PC6 Sw3 PA4*/
	GPIO_pinControlRegisterType pinControlRegisterMux1 = GPIO_MUX1;
	GPIO_pinControlRegisterType pinControlRegisterInputInterruptPSFE = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE;
	/**Configure the characteristics in the GPIOs*/
	GPIO_pinControlRegister(GPIO_B,BIT21,&pinControlRegisterMux1);//Azul
	GPIO_pinControlRegister(GPIO_B,BIT22,&pinControlRegisterMux1);//Rojo
	GPIO_pinControlRegister(GPIO_E,BIT26,&pinControlRegisterMux1);//Verde
	GPIO_pinControlRegister(GPIO_A,BIT4,&pinControlRegisterInputInterruptPSFE);//Sw3
	GPIO_pinControlRegister(GPIO_C,BIT6,&pinControlRegisterInputInterruptPSFE);//Sw2
	/**Configure Port Pins as input/output*/
	GPIO_dataDirectionPIN(GPIO_B,GPIO_OUTPUT,BIT21);
	GPIO_dataDirectionPIN(GPIO_B,GPIO_OUTPUT,BIT22);
	GPIO_dataDirectionPIN(GPIO_E,GPIO_OUTPUT,BIT26);
	GPIO_dataDirectionPIN(GPIO_C,GPIO_INPUT,BIT6);
	GPIO_dataDirectionPIN(GPIO_A,GPIO_INPUT,BIT4);
	/**Sets the threshold for interrupts, if the interrupt has higher priority constant that the BASEPRI, the interrupt will not be attended*/
	NVIC_setBASEPRI_threshold(PRIORITY_5);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTA_IRQ,PRIORITY_4);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTC_IRQ,PRIORITY_4);

	uint32 i = 1;

    while(1)
    {
		/** if comparison to read when the sw2 is pressed and be able to move between i values of 0 and 2
	 	 	 it uses the function getFC to get the value of the flag set to true when the PortC gets interrupted*/
		if(TRUE == GPIO_getFA())
			{
				GPIO_clearFA();//clears the flag of the interrupt
				if(i==1)
				{
					FunctionPoint[6]();
					FunctionPoint[0]();
					i++;
				}
				else if(i==2)
				{
					FunctionPoint[6]();
					FunctionPoint[1]();
					i++;
				}
				else if(i==3)
				{
					FunctionPoint[6]();
					FunctionPoint[2]();
					i++;
				}
				else if(i==4)
				{
					FunctionPoint[6]();
					FunctionPoint[3]();
					i++;
				}
				else if(i==5)
				{
					FunctionPoint[6]();
					FunctionPoint[4]();
					i = 1;
				}
			}
		else if(TRUE == GPIO_getFC())
		{
			GPIO_clearFC();//clears the flag of the interrupt
			if(i==1)
			{
				FunctionPoint[6]();
				FunctionPoint[4]();
				i=5;
			}
			else if(i==2)
			{
				FunctionPoint[6]();
				FunctionPoint[3]();
				i--;
			}
			else if(i==3)
			{
				FunctionPoint[6]();
				FunctionPoint[2]();
				i--;
			}
			else if(i==4)
			{
				FunctionPoint[6]();
				FunctionPoint[1]();
				i--;
			}
			else if(i==5)
			{
				FunctionPoint[6]();
				FunctionPoint[0]();
				i--;
			}
		}
		else if(TRUE == (GPIO_getFC() && GPIO_getFA()))
		{
			GPIO_clearFA();
			GPIO_clearFC();
			FunctionPoint[6]();
			FunctionPoint[5]();
		}
    }
    return 0 ;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
void delay(uint16 delay)
{
	volatile uint16 counter;

	for(counter=delay; counter > 0; counter--)
	{
	}
}
void turnLEDsOff(){
			GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			delay(1000);//65000
			GPIOB->PDOR |= 0x00400000;/**Read led off*/
			delay(1000);
			GPIOE->PDOR |= 0x4000000;/**Green led off*/
			delay(1000);
}

void blueLEDOn(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
	delay(65000);
}
void redLEDOn(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	delay(65000);
}
void greenLEDOn(){
		turnLEDsOff();
	GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
	delay(65000);
}
void yellowColor(){
		turnLEDsOff();
	GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	delay(65000);
}
void purpleColor(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	delay(65000);
}
void whiteColor()
{
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
	GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
	delay(65000);
}
