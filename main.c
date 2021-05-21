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
void ADC_start(void);

uint16_t ADC1ConvertedValue = 0;
uint16_t ADC1ConvertedVoltage = 0;

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
	DAC_init();
	ADC_init();
	ADC_start();

	TIM3->PSC = 799;  // prescalor value in Timer ‘x’ as 100
	TIM3->ARR = 9999; // Auto-Reset Register of Timer ‘x’ set to 1000

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
		ADC1->CR |= ADC_CR_ADSTART;
		// while ((ADC1->ISR & ADC_ISR_AWD2)==ADC_ISR_AWD2)
		while (!(ADC1->ISR & ADC_ISR_EOC))
			; // Test EOC flag
		if (a > 0x0000)
		{
			DAC1->DHR12R1 ^= a << 8; // turn LEds off
			GPIOE->ODR = ADC1->DR;	 // turn LEds off
									 // GPIOE->ODR ^= a << 8;	 // turn LEds off
		}
		a = a + 1;
		DAC1->DHR12R1 ^= a << 8; // toggle DAC state
		// while (!(ADC1->ISR & ADC_ISR_EOC))
		// 	;				   // Test EOC flag
		GPIOE->ODR = ADC1->DR; // turn LEds off
		// GPIOE->ODR ^= a << 8;	 // toggle LED state

		if (a > max)
		{
			a = 0x0000; // reset led counter
		}
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
							 // GPIOE->ODR ^= a;		 // toggle LED state
}

void ADC_start(void)
{
	ADC1->CR |= ADC_CR_ADSTART; // Start ADC1 Software

	// while (1)
	// {
	while (!(ADC1->ISR & ADC_ISR_EOC))
		// while ((ADC1->ISR & ADC_ISR_AWD2)==ADC_ISR_AWD2)
		;						   // Test EOC flag
	ADC1ConvertedValue = ADC1->DR; // Get ADC1 converted data
								   // ADC1ConvertedVoltage = (ADC1ConvertedValue * 3300) / 4096; // Compute the voltage
								   // }
}
void ADC_init(void)
{
	ADC1->CR &= ~ADC_CR_ADVREGEN; // reset state
	ADC1->CR |= ADC_CR_ADVREGEN_0; // enable state

	int i = 0;
	while (i <200) //delay for 10 usecond
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
	GPIOC->MODER |= 0x0000000C;		   // Configure ADC Channel7 as analog input

	// ADC configuration
	ADC1->CFGR |= ~ADC_CFGR_CONT;   // 0: Single conversion mode
	ADC1->CFGR &= ADC_CFGR_RES_1; // 8-bit data resolution
	ADC1->CFGR &= ~ADC_CFGR_ALIGN; // Right data alignment

	/* ADC1 regular channel7 configuration */
	ADC1->SQR1 |= ADC_SQR1_SQ1_2 | ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_0; // SQ1 = 0x07, start converting ch7
	ADC1->SQR1 &= ~ADC_SQR1_L;										// ADC regular channel sequence length = 0 => 1 conversion/sequence
	ADC1->SMPR1 |= ADC_SMPR1_SMP7_1 | ADC_SMPR1_SMP7_0;	 //011: 7.5 ADC clock cycles			// = 0x03 => sampling time 7.5 ADC clock cycles
	ADC1->CR |= ADC_CR_ADEN; // Enable ADC1 - SET ADEN high in ADCx_CR
	while (!ADC1->ISR & ADC_ISR_ADRD)
		; //• Wait for ADRDY flag in ADC1_ISR
}

void DAC_init()
{
	//DAC Start-up Procedure:
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
	GPIOA->MODER |= 0x00000F00; // Set ouput mode of  pin 4 and 5 to analogue mode
	// GPIOA->MODER |= 0x00000A00; // Set ouput mode of  pin 4 and 5 to analogue mode
								// Disable the ‘buffer’ function in the DAC control register
	DAC1->CR |= DAC_CR_BOFF1;
	// Enable DAC peripheral
	DAC1->CR |= DAC_CR_EN1;
}
