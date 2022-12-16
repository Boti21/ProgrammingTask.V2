#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#define BAUD 38400UL

int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);

void uart_init(void);
void io_redirect(void);


#endif

