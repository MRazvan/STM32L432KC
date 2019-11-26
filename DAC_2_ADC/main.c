#include <memory.h>
#include <stdio.h>
#include "stm32l4xx.h"

#define SYSTEM_CLOCK_SPEED 	80000000

#define LED_PIN 			3 // PB3

#define ADC_PIN				6 // PA6 -> A5 - ADC1/11
#define ADC_CHANNEL			11

#define DAC_PIN				5 // PA5 -> A4
#define DAC_CHANNEL			2 // DAC1/2

#define UART_RX				3 // PA3
#define UART_TX				2 // PA2

volatile uint32_t uwTick;
void SysTick_Handler(){
	uwTick++;
}

void initClock();
void initADC();
void initDAC();
void writeDAC(uint16_t);
void initSysTick(uint32_t);
void initLed();
void initUART(uint32_t);
void delay(uint32_t);
uint32_t readADC();
void writeMessage(char*);

char msgBuffer[200];
int main(){

	initClock();
	initSysTick(SYSTEM_CLOCK_SPEED / 1000);
	initADC();
	initDAC();
	initUART(230400);
	initLed();

	// Counter to increment and change DAC
	uint16_t count = 0;
	while(1){
		// Toggle led
		GPIOB->ODR ^= (1 << LED_PIN);

		// Update DAC
		count = (count + 10) % 4096;
		writeDAC(count);

		// Delay for 100ms
		delay(100);

		// Read ADC value and write to UART
		uint32_t val = readADC();
		memset(msgBuffer, 0, sizeof(msgBuffer));
		sprintf(msgBuffer, "ADC VAL %lu\n", val);
		writeMessage(msgBuffer);
	}
	return 0;
}

void initClock(){
	// 16 MHZ clock on MSI
	MODIFY_REG(RCC->CR, RCC_CR_MSIRANGE_Msk, RCC_CR_MSIRANGE_8);
	// CHANGE THE FREQUENCY OF MSI
	SET_BIT(RCC->CR, RCC_CR_MSIRGSEL);
	// WAIT UNTIL MSI READY
	while(READ_BIT(RCC->CR, RCC_CR_MSIRDY) == 0);

	// CONFIGURE PLL
	// PAGE 199 reference manual
	// Select MSI as the source
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk, RCC_PLLCFGR_PLLSRC_MSI);
	// M is divided by 1 -> 16Mhz
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, 0);
	// N is multiplied by 10 -> 160 Mhz
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_1);
	// R is divided by 2 -> 80 Mhz clock
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN_Msk, 0);
	// Enable R clock out
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN);

	// Enable the PLL
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	// WAIT FOR PLL TO LOCK
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0);

	// SETUP FLASH READ LATENCY, PAGE 101
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_4WS);

	// Finally switch the clock to PLL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_PLL) == 0);
}

void initUART(uint32_t baudRate){
	// Enable GPIOA
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);

	/////////////
	// RX
	// Alternate mode
	MODIFY_REG(GPIOA->MODER, 0b11 << (UART_RX * 2), 0b10 << (UART_RX * 2));
	// Pull down
	MODIFY_REG(GPIOA->PUPDR, 0b11 << (UART_RX * 2), 0b10 << (UART_RX * 2));
	// Alternate Function 7
	MODIFY_REG(GPIOA->AFR[0], 0b1111 << (UART_RX * 4), 0b0111 << (UART_RX * 4));
	/////////////
	// TX
	// Alternate mode
	MODIFY_REG(GPIOA->MODER, 0b11 << (UART_TX * 2), 0b10 << (UART_TX * 2));
	// Pull down
	MODIFY_REG(GPIOA->PUPDR, 0b11 << (UART_TX * 2), 0b10 << (UART_TX * 2));
	// Alternate Function 7
	MODIFY_REG(GPIOA->AFR[0], 0b1111 << (UART_TX*4), 0b0111 << (UART_TX*4));

	// ENABLE CLOCK FOR USART2
	MODIFY_REG(RCC->APB1ENR1, RCC_APB1ENR1_USART2EN_Msk, RCC_APB1ENR1_USART2EN);

	CLEAR_REG(USART2->CR1);
	// Configure the baud rate
	USART2->BRR = SYSTEM_CLOCK_SPEED / baudRate;
	// ENABLE THE TRANSMITTER
	SET_BIT(USART2->CR1, USART_CR1_TE);
	// ENABLE USART
	SET_BIT(USART2->CR1, USART_CR1_UE);
}

void writeMessage(char* msg) {
	while(*msg){
		// WAIT FOR TX TO BE EMPTY
		while(READ_BIT(USART2->ISR, USART_ISR_TXE) == 0);
		USART2->TDR = *msg++;
	}
}

void initADC(){
	// Enable the GPIO CLOCK for the input
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);
	// Setup the input to be Analog in
	MODIFY_REG(GPIOA->MODER, 0b11 << (ADC_PIN * 2), 0b11 << (ADC_PIN * 2));

	// PAGE 234
	// SELECT THE ADC CLOCK
	MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADCSEL_Msk, RCC_CCIPR_ADCSEL_0 | RCC_CCIPR_ADCSEL_1);
	// Enable ADC CLOCK
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_ADCEN);

	// All samples on one trigger
	CLEAR_BIT(ADC1->CFGR2, ADC_CFGR2_TROVS);
	// Enable OVERSAMPLING, Discart last 8 bits, 256x oversampling
	MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_ROVSE_Msk, ADC_CFGR2_ROVSE);
	// Discard the last 8 bits
	MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_OVSS_Msk, ADC_CFGR2_OVSS_3);
	// 256X oversampling
	MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_OVSR_Msk, ADC_CFGR2_OVSR_0 | ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_2);
	// 640.5 ADC CLOCK CYCLES SAMPLE TIME
	MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP11_Msk, ADC_SMPR2_SMP11_0 | ADC_SMPR2_SMP11_1 | ADC_SMPR2_SMP11_2);
	// For 80MHz clock we get a maximum of 80000000/(640.5*256) samples per second ~= 488 conversions / second

	// 1 Conversion
	CLEAR_REG(ADC1->SQR1);
	// Channel number
	MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1_Msk, ADC_CHANNEL << ADC_SQR1_SQ1_Pos);

	// PAGE 382
	// Disable powerdown
	CLEAR_BIT(ADC1->CR, ADC_CR_DEEPPWD);
	// Enable ADC voltage regulator
	SET_BIT(ADC1->CR, ADC_CR_ADVREGEN);

	// Delay, the datasheet specifies 20us for the Voltage regulator to start
	//		at 80mhz, 1us is 80 clock cycles, we need 1600 clock cycles for 20us
	//		just to have a margin make it 5000
	for(int16_t delayCount = 0; delayCount < 5000; ++delayCount){
		asm("nop");
	}

	// START CALIBRATION
	SET_BIT(ADC1->CR, ADC_CR_ADCAL);
	// WAIT UNTIL CALIBRATION IS DONE
	while(READ_BIT(ADC1->CR, ADC_CR_ADCAL) != 0);

	// PAGE 386
	/*
		1. Clear the ADRDY bit in the ADC_ISR register by writing ‘1’.
		2. Set ADEN=1.
		3. Wait until ADRDY=1 (ADRDY is set after the ADC startup time). This can be done
		using the associated interrupt (setting ADRDYIE=1).
		4. Clear the ADRDY bit in the ADC_ISR register by writing ‘1’ (optional).
	 */
	SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);
	SET_BIT(ADC1->CR, ADC_CR_ADEN);
	while(READ_BIT(ADC1->ISR, ADC_ISR_ADRDY) == 0);
	SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);
}

uint32_t readADC(){
	// Page 391
	// CLEAR THE EOS FLAG
	SET_BIT(ADC1->ISR, ADC_ISR_EOS);
	// START THE CONVERSION
	SET_BIT(ADC1->CR, ADC_CR_ADSTART);
	// WAIT UNTIL CONVERSION IS DONE
	while(READ_BIT(ADC1->ISR, ADC_ISR_EOS) == 0);
	return ADC1->DR;
}

void initDAC(){
	// Enable the GPIO CLOCK for the input
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);
	// Setup the input to be Analog
	MODIFY_REG(GPIOA->MODER, 0b11 << (DAC_PIN * 2), 0b11 << (DAC_PIN * 2));
	// Enable DAC CLOCK
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_DAC1EN);
	// SET TRIGGER FOR CHANNEL 2
	MODIFY_REG(DAC1->CR, DAC_CR_TSEL2_Msk, DAC_CR_TSEL2_0 | DAC_CR_TSEL2_1 | DAC_CR_TSEL2_2);
	// Enable trigger for channel 2
	MODIFY_REG(DAC1->CR, DAC_CR_TEN2_Msk, DAC_CR_TEN2);
	// Enable channel 2
	MODIFY_REG(DAC1->CR, DAC_CR_EN2_Msk, DAC_CR_EN2);
}

void writeDAC(uint16_t val){
	WRITE_REG(DAC1->DHR12R2, val);
	// Trigger dac out
	SET_BIT(DAC1->SWTRIGR, DAC_SWTRIGR_SWTRIG2);
}

void initSysTick(uint32_t ticks){

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);
  SysTick->VAL   = 0UL;
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
				   SysTick_CTRL_TICKINT_Msk   |
				   SysTick_CTRL_ENABLE_Msk;
}

void initLed(){
	// ENABLE CLOCK ON PORT B
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);
	// INPUT ON PIN 3
	MODIFY_REG(GPIOB->MODER, 0b11 << (LED_PIN * 2), 0b01 << (LED_PIN * 2));
	// LOW SPEED
	MODIFY_REG(GPIOB->OSPEEDR, 0b11 << (LED_PIN * 2), 0b00 << (LED_PIN * 2));
	// PUSH PULL
	MODIFY_REG(GPIOB->OTYPER, 0b1 << LED_PIN, 0b0 << LED_PIN);
}

void delay(uint32_t ticks){
	uint32_t current = uwTick;
	while((uwTick - current) < ticks);
}
