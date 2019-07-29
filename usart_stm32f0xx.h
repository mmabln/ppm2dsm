#ifndef USART_STM32F0XX_H_INCLUDED
#define USART_STM32F0XX_H_INCLUDED

/********************************************//**
 * \brief Init USART
 *
 * \param   ub    Pointer to USART
 * \param   baud  Baud rate to use
 * \return  Status: 0=OK, >0 error
 *
 * Initializes the USART to
 * - data format 8N1
 * - baud rate given
 *
 * The function does not take care about GPIO configuration.
 *
 ***********************************************/
uint8_t usart_init(USART_TypeDef *ub, uint32_t baud);


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
void usart_putc(USART_TypeDef *ub, char c);


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
void usart_puts(USART_TypeDef *ub, char *str);


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
void usart_write(USART_TypeDef *ub, uint8_t *data, unsigned int len);


/********************************************//**
 * \brief Get a character from the USART.
 *
 * \param   ub    Pointer to USART
 * \return  Character
 *
 * Gets a single character from the USART.
 * If there is no character available in the FIFO, it waits.
 *
 ***********************************************/
char usart_getc(USART_TypeDef *ub);

#endif /* USART_STM32F0XX_H_INCLUDED */
