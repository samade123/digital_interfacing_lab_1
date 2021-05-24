// PSC = 799
// ARR =99999
int a = 0x0000;
int max = 0x00ff;
int state = 0;
#define PIN9 0x0002u
#define PIN8 0x0001u

#include "stm32f3xx.h" // Device header

int main(void)
{

	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct pulses to clock timer
	// RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// GPIOE->MODER |= 0x00010000; // Set mode of each pin in port E
	// GPIOE->MODER |= 0x55550000;		// Set ouput mode of  pin 8-15 so all are output mode
	GPIOE->MODER |= 0x00050000;		// Set ouput mode of  pin 8-9 to output mode
	GPIOE->OTYPER &= ~(0x00000100); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x55550000);	// Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)

	TIM3->PSC = 799;  // prescalor value in Timer ‘x’ as 100
	TIM3->ARR = 9999; // Auto-Reset Register of Timer ‘x’ set to 1000 counts
	// setting timer interrupt to every 1 second

	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an
	GPIOE->ODR ^= a << 8; // turn LEds off

	NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer ‘x’ interrupt request in NVIC

	while (1)
	{
	}
}

void TIM3_IRQHandler()
{
	if ((TIM3->SR & TIM_SR_UIF) != 0) // Check interrupt source is from the ‘Update’ interrupt flag
	{
		// if (a > 0x0000)
		// {
		// 	GPIOE->ODR ^= a << 8; // turn LEds off
		// }
		// a = a + 1;
		// GPIOE->ODR ^= a << 8; // toggle LED state
		// if (a > max)
		// {
		// 	a = 0x0000; // reset led counter
		// }

		switch (state)
		{
		case 0:
			GPIOE->ODR ^= PIN8 << 8; // turn LEds off
			state = state + 1;
			break;
		case 1:
			GPIOE->ODR ^= PIN9 << 8; // turn LEds off
			state = state + 1;
			break;
		case 2:
			GPIOE->ODR ^= PIN8 << 8; // turn LEds off
			state = state + 1;
			break;
		case 3:
			GPIOE->ODR ^= PIN9 << 8; // turn LEds off
			state = 0;
			break;
		}
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
							 // GPIOE->ODR ^= a;		 // toggle LED state
}
