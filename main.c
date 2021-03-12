/* 
	Pete Hubbard 2019
	Loughborough University
	WSC055 Lab 1
	V.2.0
	
	The following 'c' code presents an outline for you to adapt during the laboratory
	
	*/

#include "stm32f3xx.h"                  // Device header


void delay(int a); // prototype for delay function

int main(void)
{
	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
	// GPIOE is a structure defined in stm32f303xc.h file
	// Define settings for each output pin using GPIOE structure
	// GPIOE->MODER |= 0x00010000; // Set mode of each pin in port E
	GPIOE->MODER |= 0x01010000; // Set ouput mode of both pin 8 and pin 12 so both are output mode
	GPIOE->OTYPER &= ~(0x00000100); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x01010000); // Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)
	
	// Main programme loop - make LED 4 (attached to pin PE.0) turn on and off	
	while (1)
  {
		int delay_value = 200000;
		GPIOE->BSRRL = 0x1100; // resets and sets the the pins 8 and 12
		delay(delay_value);
		GPIOE->BSRRH = 0x1100; 
		delay(delay_value);
	}

}

// Delay function to occupy processor
void delay (int a)
{
    volatile int i,j;

    for (i=0 ; i < a ; i++)
    {
        j++;
    }

    return;
}
