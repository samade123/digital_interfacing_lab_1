/*
	Pete Hubbard 2019
	Loughborough University
	WSC055 Lab 1
	V.2.0

	The following 'c' code presents an outline for you to adapt during the laboratory

	*/

// PSC = 799
// ARR =99999
int a = 0x0000;
int max = 0x00ff;
// int i;

#include "stm32f3xx.h" // Device header

// void TIM3_IRQHandler(void);

int main(void)
{
	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct pulses to clock timer
	//DAC Start-up Procedure:
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;

	// GPIOE->MODER |= 0x00010000; // Set mode of each pin in port E
	GPIOE->MODER |= 0x55550000;		// Set ouput mode of  pin 8-15 so all are output mode
	GPIOA->MODER |= 0x00000A00;		// Set ouput mode of  pin 4 and 5 to analogue mode
	GPIOE->OTYPER &= ~(0x00000100); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x55550000);	// Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)
									// TIM3->CR1 &= ~CEN; // Disable TIM7 interrupt

	// Disable the ‘buffer’ function in the DAC control register
	DAC1->CR |= DAC_CR_BOFF1;

	// Enable DAC peripheral
	DAC1->CR |= DAC_CR_EN1;

	// DAC1->DAC1_DHR12R1

	TIM3->PSC = 799;  // prescalor value in Timer ‘x’ as 100
	TIM3->ARR = 9999; // Auto-Reset Register of Timer ‘x’ set to 1000 counts
	// ‘Update’ Interrupt Enable (UIE) – or 0x00000001
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an

	NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer ‘x’ interrupt request in NVIC

	// Main programme loop - make LED 4 (attached to pin PE.0) turn on and off
	while (1)
	{
	}
}

void TIM3_IRQHandler()
{
	if ((TIM3->SR & TIM_SR_UIF) != 0) // Check interrupt source is from the ‘Update’ interrupt flag
	{
		if (a > 0x0000)
		{
			GPIOE->ODR ^= a << 8;		  // turn LEds off
			DAC1->DHR12R1 ^= a << 8; // turn LEds off
		}
		a = a + 1;
		GPIOE->ODR ^= a << 8;		  // toggle LED state
		DAC1->DHR12R1 ^= a << 8; // turn LEds off

		if (a > max)
		{
			a = 0x0000; // reset led counter
		}
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
							 // GPIOE->ODR ^= a;		 // toggle LED state
}
