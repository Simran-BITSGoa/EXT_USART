#include "stm32f4xx.h"
#include <stddef.h>

// Function Prototypes
void USART2_Init(void);
void GPIOC_Init(void);
void EXTI13_Init(void);
void NVIC_Init(void);
void SysTick_Init(void);
void USART_Write(char *string);
void EXTI15_10_IRQHandler(void); // Interrupt handler for EXTI lines [15:10]

// Global variables
static volatile uint32_t msTicks = 0;          // Millisecond counter (SysTick)
static volatile uint32_t last_interrupt_time = 0; // For debounce

int main(void)
{
    // Initialize peripherals
    SysTick_Init();
    USART2_Init();
    GPIOC_Init();
    EXTI13_Init();
    NVIC_Init();

    while (1)
    {
        // Main loop
    }
}

/**
  * @brief  Configure PC13 as input for an external button.
  *         Enable pull-up or pull-down as needed. Here we use pull-up.
  */
void GPIOC_Init(void)
{
    // 1. Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    // 2. Set PC13 as input
    //    Each pin has 2 bits in MODER. 00 = Input mode
    GPIOC->MODER &= ~(3UL << (13 * 2));  // Clear MODER13 (bits for PC13)
    
    // 3. Optional: Enable internal pull-up or pull-down
    //    Here we enable pull-up for demonstration:
    GPIOC->PUPDR &= ~(3UL << (13 * 2));  // Clear PUPDR13
    GPIOC->PUPDR |=  (1UL << (13 * 2));  // 01 = Pull-up
}

/**
  * @brief  Configure EXTI line 13 to be triggered on a rising edge from PC13.
  *         (Adjust to falling edge if your hardware is active-low.)
  */
void EXTI13_Init(void)
{
    // 1. Enable system configuration controller clock to access SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // 2. Map EXTI13 to PC13 in SYSCFG_EXTICR4 (covers EXTI12 to EXTI15)
    //    Each EXTI line uses 4 bits in EXTICR. For PC: the value is 0b0010.
    SYSCFG->EXTICR[3] &= ~(0xF << (4 * (13 - 12)));   // Clear bits for EXTI13
    SYSCFG->EXTICR[3] |=  (0x2 << (4 * (13 - 12)));   // 0x2 for port C
    
    // 3. Unmask EXTI line 13
    EXTI->IMR |= EXTI_IMR_MR13;
    
    // 4. Select the rising edge or falling edge trigger
    //    Here we do rising edge. If your button is active-low, use FTSR instead.
    EXTI->RTSR |= EXTI_RTSR_TR13;   // Rising trigger
    // EXTI->FTSR |= EXTI_FTSR_TR13; // Falling trigger (if needed)
}

/**
  * @brief  Enable EXTI15_10_IRQn in NVIC.
  */
void NVIC_Init(void)
{
    // Enable and set priority for EXTI line [15:10] (covers PC13)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
}

/**
  * @brief  Configure SysTick to increment msTicks every 1 ms.
  */
void SysTick_Init(void)
{
    // The exact parameter to SysTick_Config() depends on your SystemCoreClock.
    // For a default 16 MHz core clock (as an example): SystemCoreClock/1000
    SysTick_Config(SystemCoreClock / 1000);
}

/**
  * @brief  SysTick ISR - increments the global msTicks variable.
  */
void SysTick_Handler(void)
{
    msTicks++;
}

/**
  * @brief  Initialize USART2 at 9600 baud (assuming 16 MHz system clock).
  */
void USART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock

    // Configure PA2 (TX) and PA3 (RX)
    GPIOA->MODER  |= (2UL << (2 * 2)) | (2UL << (3 * 2));   // Alternate function
    GPIOA->AFR[0] |= (7UL << (4 * 2)) | (7UL << (4 * 3));   // AF7 for USART2

    // Baud rate register: 9600 baud @ 16 MHz -> 0x683
    // Adjust if your system clock differs
    USART2->BRR = 0x0683;

    // Enable USART, enable transmitter
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

/**
  * @brief  Send a null-terminated string via USART2.
  */
void USART_Write(char *string)
{
    int i;  // For C90/C99 compliance, declare outside loop
    if (string == NULL) return;
    
    for (i = 0; string[i] != '\0'; i++)
    {
        // Wait until TXE (Transmit Data Register Empty) is set
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = string[i];
    }
}

/**
  * @brief  EXTI line [15:10] interrupt handler - covers PC13 among others.
  *         Debounces using a 50 ms guard window.
  */
void EXTI15_10_IRQHandler(void)
{
    // Check if EXTI13 is pending
    if (EXTI->PR & EXTI_PR_PR13)
    {
        uint32_t current_time = msTicks;
        
        // Simple software debounce (50 ms)
        if ((current_time - last_interrupt_time) > 50)
        {
            last_interrupt_time = current_time;
            USART_Write("CMSY's Team\n");
        }
        
        // Clear the pending bit for EXTI13
        EXTI->PR |= EXTI_PR_PR13;
    }
}
