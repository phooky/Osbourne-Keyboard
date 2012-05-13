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
#include "usb_keyboard.h"

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD &= ~(1<<6))
#define LED_OFF		(PORTD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

/*****************
 Address pins - output
 Data pins - input
 * - confirmed
 A0 - D2*   D0 - F5*
 A1 - F6*   D1 - B3*
 A2 - F7*   D2 - F4*
 A3 - B6    D3 - B2
 A4 - B5    D4 - F1*
 A5 - D0*   D5 - B1
 A6 - D1*   D6 - F0
 A7 - B7    D7 - B0*
 *****************/

uint8_t keyState[8];

/****************
Keyboard matrix layout
from http://old.pinouts.ru/Inputs/osborne_keyboard_pinout.shtml
A0 Esc  Tab	Ctrl	 	Shift	Return	' "	[ ]
A1 1 !  2 @	3 £	4 $	5 %	6 ^	7 &	8 *
A2 Q    W	E	R	T	Y	U	I
A3 A    S	D	F	G	H	J	K
A4 Z    X	C	V	B	N	M	, <
A5 Up   Left	0 )	Space	. >	P	O	9 (
A6 Rgt	Down    - _	/ ?	; :	\ |	L	= +
A7  	 	 	Lock	 	 	 	 
******************/
uint8_t keyMap[64] = {
  KEY_ESC, KEY_TAB, KEY_CTRL, 0, KEY_SHIFT, KEY_ENTER, KEY_QUOTE, KEY_LEFT_BRACE,
  KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8,
  KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,
  KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,
  KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA,
  KEY_UP, KEY_LEFT, KEY_0, KEY_SPACE, KEY_PERIOD, KEY_P, KEY_O, KEY_9,
  KEY_RIGHT, KEY_DOWN, KEY_MINUS, KEY_SLASH, KEY_SEMICOLON, KEY_BACKSLASH, KEY_L, KEY_EQUAL,
  0, 0, 0, KEY_CAPS_LOCK, 0, 0, 0, 0
};

void initState(void) {
  uint8_t idx;
  for (idx = 0; idx < 8; idx++) keyState[idx] = 0;
}

void doKeyState(uint8_t a, uint8_t d, uint8_t state) {
  uint8_t oldState = (keyState[a] & _BV(d)) != 0;
  uint8_t newState = state == 0;
  uint8_t key = keyMap[(a*8)+d];
  if (key == 0) return;
  if (oldState != newState) {
    if (newState) {
      usb_keyboard_press(key, 0);
      keyState[a] |= _BV(d);
    } else {
      keyState[a] &= ~_BV(d);
    }
  }
}

void setAddr(int8_t idx) {
  // set all addr lines high
  PORTD |= _BV(2) | _BV(1) | _BV(0);
  PORTF |= _BV(7) | _BV(6);
  PORTB |= _BV(7) | _BV(6) | _BV(5);
  // set correct line low
  switch(idx) {
  case 0: PORTD &= ~_BV(2); break;
  case 1: PORTF &= ~_BV(6); break;
  case 2: PORTF &= ~_BV(7); break;
  case 3: PORTB &= ~_BV(6); break;
  case 4: PORTB &= ~_BV(5); break;
  case 5: PORTD &= ~_BV(0); break;
  case 6: PORTD &= ~_BV(1); break;
  case 7: PORTB &= ~_BV(7); break;
  }
}


void readCycle(int8_t idx) {
  setAddr(idx);
  //_delay_ms(50);
  doKeyState(idx,0,PINF & _BV(5));
  doKeyState(idx,1,PINB & _BV(3));
  doKeyState(idx,2,PINF & _BV(4));
  doKeyState(idx,3,PINB & _BV(2));
  doKeyState(idx,4,PINF & _BV(1));
  doKeyState(idx,5,PINB & _BV(1));
  doKeyState(idx,6,PINF & _BV(0));
  doKeyState(idx,7,PINB & _BV(0));
}

void initPins(void) {
  setAddr(-1);
  DDRD = _BV(2) | _BV(1) | _BV(0);
  DDRF = _BV(7) | _BV(6);
  DDRB = _BV(7) | _BV(6) | _BV(5);
}

int main(void)
{
	// set for 16 MHz clock
	CPU_PRESCALE(0);

	DDRD = 0x00;
	DDRB = 0x00;
	DDRF = 0x00;
	PORTB = 0xFF;
	PORTD = 0xFF;
	PORTF = 0xFF;
	initState();
	initPins();
	TCCR0A &= 0x03;
	TCCR1A &= 0x03;

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	while (1) {
	  uint8_t idx;
	  for (idx = 0; idx < 8; idx++) {
	    readCycle(idx);
	  }
		// now the current pins will be the previous, and
		// wait a short delay so we're not highly sensitive
		// to mechanical "bounce".
		_delay_ms(2);
	}
}


