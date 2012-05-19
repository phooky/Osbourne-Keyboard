/* Keyboard example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>


extern void patternline(uint8_t count, uint16_t base, uint8_t to_flip);
extern void usleep(uint8_t count);

uint8_t imagedata[] PROGMEM = {
  0x55, 0xAA, 0x55, 0xAA, 0xF0, 0xF5, 0xEE, 0xCC,
  0xCC, 0xEE, 0x5F, 0x0F, 0xAA, 0x55, 0xAA, 0x55
};

void fakeline(void) {
  //PORTF = _BV(1);
  PORTF = _BV(0);
  _delay_us(31);
  //PORTF = 0;
  PORTF = _BV(0) | _BV(1);
  _delay_us(32);
}

void initPins(void) {
  // Set B0,F0,F1 as outputs
  // F0 = VSYNC
  // F1 = HSYNC
  // B0 = VIDEO
  PORTF = _BV(1);
  PORTB = _BV(0);
  DDRF = _BV(0) | _BV(1);
  DDRB = _BV(0);
}

int main(void)
{
  uint8_t offset = 0;
  uint8_t direction = 1;
  CLKPR = 0x80, CLKPR = 0;
  initPins();
  while (1) {
    fakeline(); fakeline(); fakeline(); fakeline(); fakeline();
    fakeline(); fakeline(); fakeline(); fakeline(); fakeline();
    fakeline(); fakeline(); fakeline(); fakeline(); fakeline();
    //fakeline(); fakeline(); fakeline(); fakeline(); fakeline();
    unsigned char i = 0;
    for (i = 0; i < 240; i++) {
      PORTF = 0;
      if (direction) {
	if (++offset > 7) {
	  direction = 0;
	}
      } else {
	if (--offset <= 1) {
	  direction = 1;
	}
      }
      //offset = 0;
      usleep(11+offset);
      //_delay_us(11);
      // 16 bytes = 32 us
      patternline(16, (uint16_t)imagedata, 10 - offset/2);
      usleep(20-offset);
      //_delay_us(20);
      PORTB = _BV(0);
    }
  }
}


