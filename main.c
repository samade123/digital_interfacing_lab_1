
#include "stm32f3xx.h" // Device header
#include "stdbool.h"
// PSC = 799
// ARR =99999
int a = 0x0000;
int counter = 0x0000;
int max = 0x00ff;
int state, store_odr, new_odr, old_odr = 0;
bool pin8_state, pin9_state, last_pin8_state = false;
bool forward = true;
bool start = false;
#define PIN14 0x0040u
#define PIN13 0x0020u
#define PIN12 0x0010u
#define PIN11 0x0008u
#define PIN10 0x0004u
#define PIN9 0x0002u
#define PIN8 0x0001u
void ext_itr_enable(void);
void counterDecrement(void);
void counterIncrement(void);

// external connections are pin b15 to e8 and pin b0 to e9

int main(void)
{

	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable clock on GPIO port B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Enable clock on GPIO port B
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct pulses to clock timer

	// GPIOE->MODER |= 0x00010000; // Set mode of each pin in port E
	GPIOE->MODER |= 0x55550000;		// Set ouput mode of  pin 8-15 so all are output mode
	GPIOE->OTYPER &= ~(0x00000000); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x55550000);	// Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)

	GPIOB->MODER |= 0x00000000; // set all pins on port B to input mode(not actually needed) we will be using pin b0 and b1

	TIM3->PSC = 799;  // prescalor value in Timer ‘x’ as 100
	TIM3->ARR = 99999; // Auto-Reset Register of Timer ‘x’ set to 1000 counts
	// setting timer interrupt to every 1 second
	// tim_hz=Fclk/(arr+1)(psc+1)
	ext_itr_enable();

	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an
	// GPIOE->ODR ^= a << 8;		// turn LEds off

	NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer ‘x’ interrupt request in NVIC

	while (1)
	{
	}
}

void TIM3_IRQHandler()
{
	if ((TIM3->SR & TIM_SR_UIF) != 0) // Use interrupt to gnerate encoder sequence on pins 8 and 9
	{
		start = true;
		switch (state)
		{
		case 0:
			GPIOE->BSRRL = old_odr << 8; // reset odr count
			// GPIOE->ODR ^= old_odr << 8;				// reset odr count
			GPIOE->ODR ^= (PIN9 << 8) | (new_odr << 8); // turn pin 8 on and update count
			// GPIOE->ODR ^= PIN8 << 8; // turn pin 8 on
			if (forward == true)
			{

				state = state + 1;
			}
			else
			{
				state = 3;
			}
			break;
		case 1:
			// GPIOE->ODR ^= old_odr << 8;				// reset odr count
			GPIOE->BSRRL = old_odr << 8;				// reset odr count
			GPIOE->ODR ^= (PIN8 << 8) | (new_odr << 8); // turn LEds off
			// GPIOE->ODR ^= PIN9 << 8; // turn pin 0 on
			if (forward == true)
			{

				state = state + 1;
			}
			else
			{
				state = state - 1;
			}
			break;
		case 2:
			// GPIOE->ODR ^= old_odr << 8;				// reset odr count
			GPIOE->BSRRL = old_odr << 8;				// reset odr count
			GPIOE->ODR ^= (PIN9 << 8) | (new_odr << 8); // turn LEds off
			// GPIOE->ODR ^= PIN8 << 8;					// turn pin 8 off
			if (forward == true)
			{

				state = state + 1;
			}
			else
			{
				state = state - 1;
			}
			break;
		case 3:
			// GPIOE->ODR ^= old_odr << 8;				// reset odr count
			GPIOE->BSRRL = old_odr << 8;				// reset odr count
			GPIOE->ODR ^= (PIN8 << 8) | (new_odr << 8); // turn LEds off
			// GPIOE->ODR ^= PIN9 << 8;					// turn pin 9 0ff
			if (forward == true)
			{

				state = 0;
			}
			else
			{
				state = state - 1;
			}
			break;
		}
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
							 // GPIOE->ODR ^= a;		 // toggle LED state
}

void ext_itr_enable(void) //enabling interrupts on pin PB0
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable the system configuration controller to be connected to a system clock

	EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR15; // . The following unmasks EXTI0  EXTI15:

	EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR15; //. The following sets EXTI0 and EXTI15 to generate an interrupt through a rising edge:
	EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR15; //. The following sets EXTI0 and EXTI15 to generate an interrupt through a falling edge:

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;  // select exti0 as pB0
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PB; // select exti0 as pB1
	NVIC_EnableIRQ(EXTI0_IRQn);					   // Configure the NVIC to trigger the interrupt service routine
	NVIC_EnableIRQ(EXTI15_10_IRQn);				   // Configure the NVIC to trigger the interrupt service routine
	NVIC_SetPriority(EXTI0_IRQn, 100);
	NVIC_SetPriority(EXTI15_10_IRQn, 101);
}

void EXTI0_IRQHandler() //pin b0 connected to pin e9 (channel B)
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR = EXTI_PR_PR0; // clear flag*

		if (start == true)
		{

			if ((GPIOE->IDR & (PIN9 << 8)) == (PIN9 << 8))
			{ //if pin 8 is high
				pin9_state = true;
			}
			else
			{
				pin9_state = false;
			}

			if (pin8_state ^ pin9_state)
			{
				counterDecrement();
			}
			else
			{
				counterIncrement();
			}
		}
	}
};

void EXTI15_10_IRQHandler() //pin b15 connected to pin e8(channel A)
{
	if (EXTI->PR & EXTI_PR_PR15) // check source
	{
		EXTI->PR = EXTI_PR_PR15; // clear flag*

		if (start == true)
		{
			last_pin8_state = pin8_state;
			if ((GPIOE->IDR & (PIN8 << 8)) == (PIN8 << 8))
			{ //if pin 8 is high
				pin8_state = true;
			}
			else
			{
				pin8_state = false;
			}

			if (pin8_state ^ pin9_state)
			{
				counterIncrement();
			}
			else
			{
				counterDecrement();
			}
		}
	}
};

void counterIncrement(void)
{
	if (start == true)
	{
		forward = true;
		if (counter == 15)
		{
			counter = 0;
		}
		else
		{
			counter = counter + 1;
		}
	}
	old_odr = new_odr; //get previosuly on leds
	new_odr = (counter) * PIN11;
}

void counterDecrement(void)
{
	if (start == true)
	{
		forward = false;
		if (counter == 0)
		{
			counter = 15;
		}
		else
		{
			counter = counter - 1;
		}
	}
	old_odr = new_odr; //get previosuly on leds
	new_odr = (counter) * PIN11;
}
