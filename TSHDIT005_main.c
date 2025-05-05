/**
 * @file: main.c
 * @author: 
 * @student-number:
 * @ps-no:
 *
 * Student solution for practical 6.
 */

//Macros
#define STM32F051  //define target for header files, must be def'd before includes

//Includes
#include "stm32f0xx.h"											   
#include "lcd_stm32f0.h"
#include <stdint.h>

uint8_t ADC_val;
//Global Variables
uint8_t SW0_PRESSED = 0;

//Function Declerations
void ResetClockTo48Mhz(void);
void init_student(void);
void init_ADC(void);
void init_GPIOB(void);
void init_GPIOA(void);
void init_TIM3 (void);

//Main Function
int main (void){
    ResetClockTo48Mhz();
    init_student();
    init_GPIOB();
    init_GPIOA();
    init_ADC();
    while (1){}
}						

//Function Definitions
/**
 * Function resets the STM32 Clocks to 48 MHz
 */
void ResetClockTo48Mhz(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
    {
        RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    }
    RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
    while ((RCC->CR & RCC_CR_PLLRDY) != 0);
    RCC->CFGR = ((RCC->CFGR & (~0x003C0000)) | 0x00280000);
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void init_student(void)
{
    init_LCD();
    lcd_command(CLEAR);
    lcd_putstring("TSHDIT005");
}


void init_ADC(void)
{
    // Enable clock for Port A
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    // Set PA6 to analogue mode 
    GPIOA -> MODER |= GPIO_MODER_MODER6;

    // Enable clock for ADC
    RCC -> APB2ENR |= RCC_APB2ENR_ADCEN;

    // Stop the ADC, before we set configuration
    ADC1 -> CR &= ~ADC_CR_ADSTART;

    // Select channel 6
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL6;

    // Configure to 8 bit
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;
    ADC1 -> CFGR1 |= ADC_CFGR1_RES_1;

    // Left aligned
    ADC1 -> CFGR1 |= ADC_CFGR1_ALIGN;


    //Wait mode
    ADC1->CFGR1 |=  ADC_CFGR1_WAIT;

    ADC1->CR |= ADC_CR_ADEN; // Set ADEN=1 in ADC_CR register, starts ADC
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0); // Wait for ADC to be ready to start converting

    // Start the ADC conversion
    ADC1 -> CR |= ADC_CR_ADSTART;
    // Conversion is complete when the EOC flag goes high in the ISR
    while( ((ADC1 -> ISR) & ADC_ISR_EOC) == 0 );

    // Enable interrupt on End of Conversion
    ADC1->IER |= ADC_IER_EOCIE;

    // Enable ADC interrupt in NVIC
    NVIC_EnableIRQ(ADC1_IRQn);

}

void init_GPIOB(void)
{
    RCC ->AHBENR |= RCC_AHBENR_GPIOBEN;     // enable GPIOB 
    GPIOB -> MODER |= GPIO_MODER_MODER0_0;   // PB0 to output mode
    GPIOB -> MODER |= GPIO_MODER_MODER1_0;
    GPIOB -> MODER |= GPIO_MODER_MODER2_0;
    GPIOB -> MODER |= GPIO_MODER_MODER3_0;
    GPIOB -> MODER |= GPIO_MODER_MODER4_0;
    GPIOB -> MODER |= GPIO_MODER_MODER5_0;
    GPIOB -> MODER |= GPIO_MODER_MODER6_0;
    GPIOB -> MODER |= GPIO_MODER_MODER7_0;   // PB7 to output mode
}

void init_GPIOA(void)
{
    // Enable SW0
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;    //enable GPIOA
    GPIOA -> MODER &= ~GPIO_MODER_MODER0;   // Set PA0 to input mode 

    GPIOA -> PUPDR &= ~GPIO_PUPDR_PUPDR0;   // Clear "_____00"
    GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR0_0;  // interface PA0 to a pull up resistor

}

void ADC1_COMP_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_EOC) // Check if End Of Conversion flag
    {
        // Clear EOC flag
        ADC1->ISR |= ADC_ISR_EOC;
        // Read 8-bit ADC value
        ADC_val = (uint8_t)(ADC1->DR >> 8);

        if (SW0_PRESSED == 0)
        {
            // Show ADC value on PB0–PB7 (LEDs)
            GPIOB->ODR = ADC_val;
        }
        // Latch SW0 press
        if ((GPIOA->IDR & GPIO_IDR_0) == 0 && SW0_PRESSED == 0)
        {
            SW0_PRESSED = 1;
            GPIOB->ODR = 0; // Turn off all LEDs (clear output register)

        }
    }
    // Start the ADC conversion
    ADC1 -> CR |= ADC_CR_ADSTART;
}

void init_TIM3 (void) 
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3 -> PSC = 0;    
    TIM3 -> ARR = 255;  

}

void TIM3_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_EOC) { // Check if End Of Conversion flag is set

        // Read the 8-bit ADC value
        ADC_val = (uint8_t)(ADC1->DR >> 8);

        // If SW0 is not pressed, update the LEDs (PB0-PB7)
        if (SW0_PRESSED == 0) 
        {
            GPIOB->ODR = ADC_val;  // Update LED outputs with ADC value
        }
    }
}
