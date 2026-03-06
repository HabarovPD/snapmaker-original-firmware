/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include "Marlin.h"
#include "MarlinSerial.h"


ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer = { { 0 }, 0, 0 };


// Constructors ////////////////////////////////////////////////////////////////

MarlinSerial::MarlinSerial() { }

// Public Methods //////////////////////////////////////////////////////////////

void MarlinSerial::begin(long baud) 
{
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate = baud;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(THIS_UART, &USART_InitStruct);
	USART_Cmd(THIS_UART, ENABLE);
	USART_ITConfig(THIS_UART, USART_IT_RXNE, ENABLE);

	
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
}

void MarlinSerial::end() {

}


int MarlinSerial::peek(void) {
  int v;
  CRITICAL_SECTION_START;
  uint16_t t = rx_buffer.tail;
  if (rx_buffer.head == t) {
    v = -1;
  }
  else {
    v = rx_buffer.buffer[t];
  }
  CRITICAL_SECTION_END;
  return v;
}

int MarlinSerial::read(void) {
  int v;
  //CRITICAL_SECTION_START;
  uint16_t t = rx_buffer.tail;
  uint16_t h = rx_buffer.head;
  if (h == t) {
    v = -1;
  }
  else {
    v = rx_buffer.buffer[t];
    rx_buffer.tail = (uint16_t)(t + 1) % (RX_BUFFER_SIZE);
  }
  //CRITICAL_SECTION_END;
  return v;
}

void MarlinSerial::flush() {
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  CRITICAL_SECTION_START;
    rx_buffer.head = rx_buffer.tail;
  CRITICAL_SECTION_END;
}


/// imports from print.h


void MarlinSerial::print(char c, int base) {
  print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base) {
  print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base) {
  print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base) {
  print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base) {
  if (base == 0) {
    write(n);
  }
  else if (base == 10) {
    if (n < 0) {
      print('-');
      n = -n;
    }
    printNumber(n, 10);
  }
  else {
    printNumber(n, base);
  }
}

void MarlinSerial::print(unsigned long n, int base) {
  if (base == 0) write(n);
  else printNumber(n, base);
}

void MarlinSerial::print(double n, int digits) {
  printFloat(n, digits);
}

void MarlinSerial::println(void) {
  print('\r');
  print('\n');
}


void MarlinSerial::println(const char c[]) {
  print(c);
  println();
}

void MarlinSerial::println(char c, int base) {
  print(c, base);
  println();
}

void MarlinSerial::println(unsigned char b, int base) {
  print(b, base);
  println();
}

void MarlinSerial::println(int n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned int n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(long n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned long n, int base) {
  print(n, base);
  println();
}

void MarlinSerial::println(double n, int digits) {
  print(n, digits);
  println();
}

// Private Methods /////////////////////////////////////////////////////////////

void MarlinSerial::printNumber(unsigned long n, uint8_t base) {
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char)(buf[i - 1] < 10 ?
                 '0' + buf[i - 1] :
                 'A' + buf[i - 1] - 10));
}

void MarlinSerial::printFloat(double number, uint8_t digits) {
  // Handle negative numbers
  if (number < 0.0) {
    print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) print('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}
// Preinstantiate Objects //////////////////////////////////////////////////////


MarlinSerial customizedSerial;

extern "C"
{
void USART1_IRQHandler(void)
{
	uint8_t recvdata;
	uint16_t h;
	uint16_t i;
	if(THIS_UART->SR & USART_FLAG_RXNE)
	{
		recvdata = THIS_UART->DR;
		h = rx_buffer.head;
		i = (h + 1)  % (RX_BUFFER_SIZE);

		if (i != rx_buffer.tail) 
		{
			rx_buffer.buffer[h] = recvdata;
			rx_buffer.head = i;
		}
	}
	else if(THIS_UART->SR & USART_FLAG_TXE)
	{
		i = tx_buffer.tail;
		h = tx_buffer.head;
		if(i == h)
		{
			THIS_UART->CR1 &= ~USART_CR1_TXEIE;
			THIS_UART->SR &= USART_FLAG_TXE;
		}
		else
		{
			THIS_UART->DR = tx_buffer.buffer[i];
			tx_buffer.tail = (i + 1) % sizeof(tx_buffer.buffer);
		}
	}
}

}



