#include "uart.h"
#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

void uart_init(void) {
    uint16_t ubrr=(uint16_t)MYUBRR;
    UBRR0H=(uint8_t)(ubrr>>8);
    UBRR0L=(uint8_t)ubrr;
    UCSR0B=(1<<RXEN0)|(1<<TXEN0);
    UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);
}
void uart_transmit(char data) {
    while(!(UCSR0A&(1<<UDRE0)));
    UDR0=data;
}
void uart_print(const char *s) {
    while(*s) uart_transmit(*s++);
}
