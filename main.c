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
void ADC_init(void);
void DAC_init(void);
void intr_btn(void);

int main(void)
{

	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	// RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct pulses to clock timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// GPIOE->MODER |= 0x00010000; // Set mode of each pin in port E
	// GPIOE->MODER |= 0x55550000;		// Set ouput mode of  pin 8-15 so all are output mode
	GPIOE->MODER |= 0x08880000;		// Set ouput mode of  pin 9-10 to alternate function mode
	GPIOE->OTYPER &= ~(0x00000100); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x55550000);	// Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)
									// TIM3->CR1 &= ~CEN; // Disable TIM7 interrupt

	GPIOE->AFR[1] |= 0x0202020; //Setting PE.8-10 to output TIM1_CH1 is obtained using the following command
	intr_btn();

	TIM1->PSC = 799; // prescalor value in Timer ‘x’ as 100
	TIM1->ARR = 9;	 // Auto-Reset Register of Timer ‘x’ set to 1000
	// 100hz = 8MHZ/(9+1)*(799+1)

	TIM1->CCMR1 |= 0x00006060; // The followed sets channel 1-2 to be standard PWM mode:
	TIM1->CCMR2 |= 0x00000060; // The followed sets channel 3 to be standard PWM mode:
	TIM1->CCR1 = 1;			   //Sets on time to 10 clock pulses
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 9;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; //. The following enables output for Channel 1:s

	TIM1->CR1 |= TIM_CR1_CEN; //Enable Timer

	while (1)
	{
	}
}

void intr_btn(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable the system configuration controller to be connected to a system clock
	EXTI->IMR |= EXTI_IMR_MR0;			  // . The following unmasks EXTI0:
	EXTI->RTSR |= EXTI_RTSR_TR0;		  //. The following sets EXTI0 to generate an interrupt through a rising edge:
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	NVIC_EnableIRQ(EXTI0_IRQn); // Enable Timer ‘x’ interrupt request in NVIC
	NVIC_SetPriority(EXTI0_IRQn, 100);
}

void EXTI0_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR |= EXTI_PR_PR0; // clear flag*
		if (TIM1->CCR1 == 10) // increment intensity of led.
		{
			TIM1->CCR1 = 0;
		}
		else
		{
			TIM1->CCR1 += 1;
		}
	}
};

