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
void ADC_init();
void DAC_init();

int main(void)
{

	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct pulses to clock timer

	// GPIOE->MODER |= 0x00010000; // Set mode of each pin in port E
	GPIOE->MODER |= 0x55550000;		// Set ouput mode of  pin 8-15 so all are output mode
	GPIOE->OTYPER &= ~(0x00000100); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x55550000);	// Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)
									// TIM3->CR1 &= ~CEN; // Disable TIM7 interrupt
	ADC_init(void);
	DAC_init(void);

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
			GPIOE->ODR ^= a << 8;	 // turn LEds off
			DAC1->DHR12R1 ^= a << 8; // turn LEds off
		}
		a = a + 1;
		GPIOE->ODR ^= a << 8;	 // toggle LED state
		DAC1->DHR12R1 ^= a << 8; // toggle DAC state

		if (a > max)
		{
			a = 0x0000; // reset led counter
		}
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
							 // GPIOE->ODR ^= a;		 // toggle LED state
}

void ADC_init(void)
{
	ADC1->CR &= ~ADC_CR_ADVREGEN;
	ADC1->CR |= ADC_CR_ADVREGEN_0;

	int i = 0;
	while (i < 100) //delay for 10 usecond
	{
		// cout << i << "\n";
		i++;
	}

	ADC1->CR &= ~ADC_CR_ADCALDIF; // calibration in Single-ended inputs Mode.
	ADC1->CR |= ADC_CR_ADCAL;	  // Start ADC calibration
	// Read at 1 means that a calibration in progress.
	while (ADC1->CR & ADC_CR_ADCAL)
		; // wait until calibration done
	// calibration_value = ADC1->CALFACT; // Get Calibration Value ADC1

	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2; // Configure the ADC clock
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;	   // Enable ADC1 clock
	ADC1_2_COMMON->CCR |= 0x00010000;

	// ADC Channel configuration PC1 in analog mode
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC Periph clock enable
	GPIOC->MODER |= 0x00000004;		   // Configure ADC Channel7 as analog input

	// ADC configuration
	ADC1->CFGR |= ADC_CFGR_CONT;   // ADC_ContinuousConvMode_Enable
	ADC1->CFGR &= ~ADC_CFGR_RES_1;   // 8-bit data resolution
	ADC1->CFGR &= ~ADC_CFGR_ALIGN; // Right data alignment
}

void DAC_init()
{
	//DAC Start-up Procedure:
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
	GPIOA->MODER |= 0x00000A00; // Set ouput mode of  pin 4 and 5 to analogue mode
								// Disable the ‘buffer’ function in the DAC control register
	DAC1->CR |= DAC_CR_BOFF1;
	// Enable DAC peripheral
	DAC1->CR |= DAC_CR_EN1;
}
