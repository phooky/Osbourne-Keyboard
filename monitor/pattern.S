#include <avr/io.h>

/* r0 - temp */
/* r24 count */
/* r22 - r23 location */
/* r18 - sreg temp store */
.global patternline

patternline:	
	movw r30, r22 		; 1  init mem location
	clr r0			; 1  clear first bit
loop:
	;; -- BIT 0 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
/*
	ld r0, Z+		    ; 3 load next byte
	*/
	ldi r18, 0x11		    ; load imm
	com r18
	mov r0, r18
	;; -- BIT 1 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	clt			    ; 1 clear T
	nop			    ; 1 snore
	;; -- BIT 2 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	dec r24			    ; 1 decrement count
	in r18, _SFR_IO_ADDR(SREG)  ; 1 copy status register
	;; -- BIT 3 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	bst r18, 1		    ; 1 store cached zero flag in T
	nop			    ; 1 snore
	;; -- BIT 4 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	nop			    ; 1 snore
	nop			    ; 1 snore
	;; -- BIT 5 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	nop			    ; 1 snore
	nop			    ; 1 snore
	;; -- BIT 6 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	nop			    ; 1 snore
	nop			    ; 1 snore
	;; -- BIT 7 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	brtc loop		    ; 2 if true -- loop (1 if false)

	ret			; 4 cycles

