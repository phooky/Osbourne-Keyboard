#include <avr/io.h>

;;; Sleep for the specified number of microseconds.
;;; The minimum sleep is 1uS. The sleep accounts
;;; for the call and return, but _not_ the time
;;; required to set up the parameters.
.global usleep
	;; r24 : time in uS
usleep:
	dec r24
	clc
	rol r24			;r24 now in .5uS
	rol r24			;r24 now in .25uS
sl_loop:
	nop
	dec r24
	brne sl_loop		;2 clocks
	nop
	ret

;;; Draw a pattern from program memory to the
;;; video output. Two blank pixels will be drawn
;;; at the start of the pattern.
.global patternline
	;; Parameters:
	;; r24 : number of pixels to draw (after blanks)
	;; r22 : location of pattern to draw (uint16_t)
	;; r20 : number of bytes through the pattern to
	;;       toggle the HSync pin

	;; Variables: 
	;; r0  : buffer for currently drawn byte
	;; r18 : sreg temp store for count
	;; r19 : sreg temp store for midpoint flip

patternline:	
	movw r30, r22 		; 1  init mem location
	clr r0			; 1  clear first bit
	com r0			; 1  set first bit
loop:
	;; -- BIT 0 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	lpm r0, Z+		    ; 3 load next byte
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
	dec r20			    ; 1 decrement flip
	in r19, _SFR_IO_ADDR(SREG)  ; 1 copy status register
	;; -- BIT 4 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	nop			    ; 1 snore
	bst r19, 1		    ; 1 store flip cached zero flag in T
	;; -- BIT 5 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	brts flip                   ; 2 if true - jump to flip variant (1 else) 
	nop			    ; 1 snore
	;; -- BIT 6 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	bst r18, 1		    ; 1 store cached zero flag in T
	nop			    ; 1 snore
	;; -- BIT 7 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	brtc loop		    ; 2 if true -- loop (1 if false)

	ret			; 4 cycles
	;; Alternate bit 6 and 7 that flips the hsync bit
flip:	
	;; -- BIT 6 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	sbi _SFR_IO_ADDR(PORTF), 1  ; 1 flip horiz
	bst r18, 1		    ; 1 store cached zero flag in T
	;; -- BIT 7 --
	out _SFR_IO_ADDR(PORTB), r0 ; 1 write bit
	asr r0			    ; 1 shift right
	brtc loop		    ; 2 if true -- loop (1 if false)
	ret			    ; 4 cycles