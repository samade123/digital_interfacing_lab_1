
#include "stm32f3xx.h" // Device header
#include "stdbool.h"
// PSC = 799
// ARR =99999
int a = 0x0000;
int counter, last_thruster, ADC1ConvertedValue, pot_odr, pot_old_odr = 0x0000;
int max = 0x00ff;
int state, store_odr, new_encoder_odr, old_encoder_odr, thruster_pos = 0;
bool pin8_state, pin9_state, last_pin8_state = false;
bool forward, thruster_forward, encoder_forward = true;
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
void thruster_position(void);
void check_encoder_pos(void);
void ADC_init(void);
void DAC_init(void);
void ADC_start(void);

// external connections are pin b15 to e8 and pin b0 to e9

int main(void)
{

	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable clock on GPIO port B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable clock on GPIO port B
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct pulses to clock timer

	GPIOE->MODER |= 0x55550000;		// Set ouput mode of  pin 8-15 so all are output mode
	GPIOE->OTYPER &= ~(0x00000000); // Set output type for each pin required in Port E(open drain for pin 8 and push pull for pin 12)
	GPIOE->PUPDR &= ~(0x55550000);	// Set Pull up/Pull down resistor configuration for Port E(pin 8 and pin 12 both set to pull up resistor)
	GPIOB->MODER |= 0x00000000;		// set all pins on port B to input mode(not actually needed) we will be using pin b0 and b1

	TIM3->PSC = 799;  // prescalor value in Timer ‘x’ as 100
	TIM3->ARR = 9999; // Auto-Reset Register of Timer ‘x’ set to 1000 counts
	// setting timer interrupt to every 1 second
	// tim_hz=Fclk/(arr+1)(psc+1)
	ext_itr_enable();

	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an
	DAC_init();
	ADC_init();

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
		thruster_position(); //run virtual encoder
		check_encoder_pos(); // check which direction thruster moving for encoder
		ADC_start();
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
}
void thruster_position(void) //woring fine
{							 //genrate traingular waveform to simulate thruster positon
	if (thruster_forward == true)
	{
		thruster_pos = thruster_pos + 1;
		if (thruster_pos >= 79)
		{
			thruster_forward = false;
		}
	}
	else
	{
		thruster_pos = thruster_pos - 1;
		if (thruster_pos <= 0)
		{
			thruster_forward = true;
		}
	}
}

void check_encoder_pos(void)
{
	if (thruster_pos != last_thruster)
	{
		encoder_forward = thruster_forward;

		switch (state)
		{
		case 0:
			// GPIOE->BSRRH = (PIN9 << 8) | (PIN8 << 8) | (old_encoder_odr << 8); // reset odr count
			// GPIOE->BSRRL = (new_encoder_odr << 8);
			if (encoder_forward == true)
			{

				state = state + 1;
			}
			else
			{
				state = 3;
			}
			break;
		case 1:
			// GPIOE->BSRRH = (PIN9 << 8) | (old_encoder_odr << 8); // reset odr count
			// GPIOE->BSRRL = (PIN8 << 8) | (new_encoder_odr << 8); // turn pin 8 on and update count
			if (encoder_forward == true)
			{

				state = state + 1;
			}
			else
			{
				state = state - 1;
			}
			break;
		case 2:
			// GPIOE->BSRRH = (old_encoder_odr << 8);							   // reset odr count
			// GPIOE->BSRRL = (PIN8 << 8) | (PIN9 << 8) | (new_encoder_odr << 8); // turn LEds off
			if (encoder_forward == true)
			{

				state = state + 1;
			}
			else
			{
				state = state - 1;
			}
			break;
		case 3:
			// GPIOE->BSRRH = (PIN8 << 8) | (old_encoder_odr << 8); // reset odr count
			// GPIOE->BSRRL = (PIN9 << 8) | (new_encoder_odr << 8); // turn LEds off
			if (encoder_forward == true)
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
}

void ext_itr_enable(void) //enabling interrupts on pin PB0
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable the system configuration controller to be connected to a system clock

	EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR9 | EXTI_IMR_MR15; // . The following unmasks EXTI0  EXTI15:

	EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR9 | EXTI_RTSR_TR15; //. The following sets EXTI0 and EXTI15 to generate an interrupt through a rising edge:
	EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR9 | EXTI_FTSR_TR15; //. The following sets EXTI0 and EXTI15 to generate an interrupt through a falling edge:

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;  // select exti0 as pB0
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PB;  // select exti0 as pB1
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PB; // select exti0 as pB1
	NVIC_EnableIRQ(EXTI0_IRQn);					   // Configure the NVIC to trigger the interrupt service routine
	NVIC_EnableIRQ(EXTI9_5_IRQn);				   // Configure the NVIC to trigger the interrupt service routine
	NVIC_EnableIRQ(EXTI15_10_IRQn);				   // Configure the NVIC to trigger the interrupt service routine
	NVIC_SetPriority(EXTI0_IRQn, 100);
	NVIC_SetPriority(EXTI9_5_IRQn, 101);
	NVIC_SetPriority(EXTI15_10_IRQn, 102);
	// EXTI9_5_IRQn
}

void EXTI0_IRQHandler() //pin b0 connected to pin e9 (channel B)
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR = EXTI_PR_PR0; // clear flag*
	}
};

void EXTI9_5_IRQHandler() //pin b9 connected to pin e9 (channel B)
{
	if (EXTI->PR & EXTI_PR_PR9) // check source
	{
		EXTI->PR = EXTI_PR_PR9; // clear flag*

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
		// forward = true;
		if (counter == 511)
		{
			counter = 0;
		}
		else
		{
			counter = counter + 1;
		}
	}
	old_encoder_odr = new_encoder_odr; //get previosuly on leds
	new_encoder_odr = (counter >> 4) * PIN11;
}

void counterDecrement(void)
{
	if (start == true)
	{
		// forward = false;
		if (counter == 0)
		{
			counter = 511;
		}
		else
		{
			counter = counter - 1;
		}
	}
	old_encoder_odr = new_encoder_odr;		  //get previosuly on leds
	new_encoder_odr = (counter >> 4) * PIN11; //to display on 5 leds
}

void DAC_init()
{
	//DAC Start-up Procedure:
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
	GPIOA->MODER |= 0x00000FF0; // Set ouput mode of  pin 4 and 5 to analogue mode
	// Disable the ‘buffer’ function in the DAC control register
	DAC1->CR |= DAC_CR_BOFF1;
	// Enable DAC peripheral
	DAC1->CR |= DAC_CR_EN1;
}
void ADC_start(void) //connect PC1 to PA4/5
{
	ADC1->CR |= ADC_CR_ADSTART;
	while (!(ADC1->ISR & ADC_ISR_EOC))
		; // Test EOC flag
	DAC1->DHR12R1 = thruster_pos << 8;
	pot_odr = (ADC1->DR >> 2) * PIN11;

	GPIOE->BSRRH = pot_old_odr << 8; // reset odr count
	GPIOE->BSRRL = pot_odr << 8; // turn LEds off
	// GPIOE->ODR = pot_odr << 8; // turn LEds off
	pot_old_odr = pot_odr;
}

// DAC1->DHR12R1 ^= a << 8; // toggle DAC state

void ADC_init(void)
{
	ADC1->CR &= ~ADC_CR_ADVREGEN;  // reset state
	ADC1->CR |= ADC_CR_ADVREGEN_0; // enable state

	int i = 0;
	while (i < 200) //delay for 10 usecond
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
	// RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC Periph clock enable
	GPIOC->MODER |= 0x0000000C; // Configure ADC Channel7 as analog input

	// ADC configuration
	ADC1->CFGR |= ~ADC_CFGR_CONT;  // 0: Single conversion mode
	ADC1->CFGR &= ADC_CFGR_RES_1;  // 8-bit data resolution
	ADC1->CFGR &= ~ADC_CFGR_ALIGN; // Right data alignment

	/* ADC1 regular channel7 configuration */
	ADC1->SQR1 |= ADC_SQR1_SQ1_2 | ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_0; // SQ1 = 0x07, start converting ch7
	ADC1->SQR1 &= ~ADC_SQR1_L;										// ADC regular channel sequence length = 0 => 1 conversion/sequence
	ADC1->SMPR1 |= ADC_SMPR1_SMP7_1 | ADC_SMPR1_SMP7_0;				//011: 7.5 ADC clock cycles			// = 0x03 => sampling time 7.5 ADC clock cycles
	ADC1->CR |= ADC_CR_ADEN;										// Enable ADC1 - SET ADEN high in ADCx_CR
	while (!ADC1->ISR & ADC_ISR_ADRD)
		; //• Wait for ADRDY flag in ADC1_ISR
}