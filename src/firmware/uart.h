/*
 * bbshd-fw
 *
 * Copyright (C) Daniel Nilsson, 2021.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _UART_H_
#define _UART_H_
#include "stc15.h"
#include "interrupt.h"
#include <stdint.h>

void uart1_open(uint32_t baudrate);
void uart2_open(uint32_t baudrate);

void uart1_close();
void uart2_close();

uint8_t uart1_available();
uint8_t uart2_available();

uint8_t uart1_read();
uint8_t uart2_read();

void uart1_write(uint8_t byte);
void uart2_write(uint8_t byte);

void uart1_flush();
void uart2_flush();


INTERRUPT_USING(isr_uart1, IRQ_UART1, 3);
INTERRUPT_USING(isr_uart2, IRQ_UART2, 3);

#endif
