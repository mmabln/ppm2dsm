#include "stm32f0xx.h"
#include "usart_stm32f0xx.h"


static uint8_t ppre_div[8] = {1, 1, 1, 1, 2, 4, 8, 16};


/********************************************//**
 * \brief Init USART
 *
 * \param   ub    Pointer to USART
 * \param   baud  Baud rate to use
 * \return  Status: 0=OK, >0 error
 *
 * Initializes the USART to
 * - data format 8N1
 * - baud rate given using PCLK clock
 *
 * The function does not take care about GPIO configuration.
 *
 ***********************************************/
uint8_t usart_init(USART_TypeDef *ub, uint32_t baud)
{
   uint32_t ppreclk;

   // check if baud rate is valid

   if(baud == 0) {
         return 1;
   }

   ppreclk = SystemCoreClock/ppre_div[(RCC->CFGR&RCC_CFGR_PPRE)>>8];

   if(ppreclk/baud > 0xffff) {
         return 1;
   }

   // Enable peripheral and set clock to PCLK

   if(ub == USART1) {
      RCC->CFGR3 &= ~(RCC_CFGR3_USART1SW);
      RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
   }
   else if(ub == USART2) {
      RCC->CFGR3 &= ~(RCC_CFGR3_USART2SW);
      RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
   }
   else {
      return 1;
   }

   // Setup baud rate generator

   ub->BRR &= ~0xffff;
   ub->BRR |= ppreclk/baud;

   // Set data format and operation

   // All default values are OK!

   // Enable the USART

   ub->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

   return 0;
}


/********************************************//**
 * \brief Send character out on USART.
 *
 * \param   ub    Pointer to USART
 * \param   c     Character to send
 * \return
 *
 * Sends a single character out on USART. Will wait until sending is
 * finished.
 *
 ***********************************************/
void usart_putc(USART_TypeDef *ub, char c)
{
   // send character

   ub->TDR = c;

   // wait until completetion

   while((ub->ISR & USART_ISR_TC) == 0);
}


/********************************************//**
 * \brief Send null terminated string out on USART.
 *
 * \param   ub    Pointer to USART
 * \param   str   Pointer to string
 * \return
 *
 * Sends a null terminated string out on USART. Will wait until sending is
 * finished.
 *
 ***********************************************/
void usart_puts(USART_TypeDef *ub, char *str)
{
   char c;

   while((c = *str++)) {

      // send character

      ub->TDR = c;

      // wait until put in shift register

      while((ub->ISR & USART_ISR_TXE) == 0);
   }

   // wait until send complete

   while((ub->ISR & USART_ISR_TC) == 0);
}


/********************************************//**
 * \brief Send data out on USART.
 *
 * \param   ub    Pointer to USART
 * \param   data  Pointer to data
 * \param   len   Length of data
 *
 * \return
 *
 * Sends data out on USART. Will wait until sending is
 * finished.
 *
 ***********************************************/
void usart_write(USART_TypeDef *ub, uint8_t *data, unsigned int len)
{
   while(len > 0) {

      // send character

      ub->TDR = *data++;

      // wait until put in shift register

      while((ub->ISR & USART_ISR_TXE) == 0);

      len--;
   }

   // wait until send complete

   while((ub->ISR & USART_ISR_TC) == 0);
}


/********************************************//**
 * \brief Get a character from the USART.
 *
 * \param   ub    Pointer to USART
 * \return  Character
 *
 * Gets a single character from the USART.
 * If there is no character available it waits.
 *
 ***********************************************/
char usart_getc(USART_TypeDef *ub)
{
   // wait for character being available

   while((ub->ISR & USART_ISR_RXNE) == 0);

   // get character

   return ub->RDR;
}
