#ifndef UART_H
#define UART_H
#include <avr/io.h>
void uart_init(void);
void uart_transmit(char data);
void uart_print(const char *str);
#endif
