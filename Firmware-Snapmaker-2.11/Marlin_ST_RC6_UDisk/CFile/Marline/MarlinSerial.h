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
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h
#include "Marlin.h"
#include "stm32f10x_conf.h"


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  EnterCritical(1);
  #define CRITICAL_SECTION_END    EnterCritical(0);
#endif


#define THIS_UART	USART1



#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define RX_BUFFER_SIZE	512

struct ring_buffer {
  unsigned char buffer[RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
};


extern ring_buffer rx_buffer;
extern ring_buffer tx_buffer;

class MarlinSerial { //: public Stream

  public:
    MarlinSerial();
    void begin(long);
    void end();
    int peek(void);
    int read(void);
    void flush(void);

    FORCE_INLINE uint16_t available(void) {
      CRITICAL_SECTION_START;
        uint16_t h = rx_buffer.head;
        uint16_t t = rx_buffer.tail;
      CRITICAL_SECTION_END;
      return (uint16_t)(RX_BUFFER_SIZE + h - t) % (RX_BUFFER_SIZE);
    }

	FORCE_INLINE void write(uint8_t c) {
      while (!(THIS_UART->SR & USART_FLAG_TXE));
      THIS_UART->DR = c;
    }

    FORCE_INLINE void checkRx(void) {
      if (THIS_UART->SR & USART_FLAG_RXNE) {
        unsigned char c  =  THIS_UART->DR;
        //CRITICAL_SECTION_START;
          uint16_t h = rx_buffer.head;
          uint16_t i = (uint16_t)(h + 1) % (RX_BUFFER_SIZE);

          // if we should be storing the received character into the location
          // just before the tail (meaning that the head would advance to the
          // current location of the tail), we're about to overflow the buffer
          // and so we don't write the character or advance the head.
          if (i != rx_buffer.tail) {
            rx_buffer.buffer[h] = c;
            rx_buffer.head = i;
          }
        //CRITICAL_SECTION_END;
      }
    }

  private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);

  public:
    FORCE_INLINE void write(const char* str) { while (*str) write(*str++); }
	
    FORCE_INLINE void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
    FORCE_INLINE void print(const char* str) { write(str); }

    void print(char, int = 0);
    void print(unsigned char, int = 0);
    void print(int, int = 10);
    void print(unsigned int, int = 10);
    void print(long, int = 10);
    void print(unsigned long, int = 10);
    void print(double, int = 2);

    void println(const char[]);
    void println(char, int = 0);
    void println(unsigned char, int = 0);
    void println(int, int = 10);
    void println(unsigned int, int = 10);
    void println(long, int = 10);
    void println(unsigned long, int = 10);
    void println(double, int = 2);
    void println(void);
};

extern MarlinSerial customizedSerial;

#endif
