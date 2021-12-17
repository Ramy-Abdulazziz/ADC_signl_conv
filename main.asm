;
; ADC_sgnl_conv.asm


.equ PERIOD = 100 ; Put correct number in for 40.0Hz
	
.dseg
	bcd_entries: .byte 4 
	led_display: .byte 4
	digit_num:	 .byte 1
.cseg
reset:
 	jmp init			;reset vector executed a power ON


.org TCA0_OVF_vect
	jmp toggle_post_ISR:



init: 
	//Ports A and D are outputs 
	ldi r16, 0xFF
	sts PORTD_DIR, r16
	sts PORTA_DIR, r16
	//WGMODE NORMAL
	ldi r16, TCA_SINGLE_WGMODE_NORMAL_gc	
	sts TCA0_SINGLE_CTRLB, r16

	//ENABLE OVERFLOW INTERUP
	ldi r16, TCA_SINGLE_OVF_bm		
	sts TCA0_SINGLE_INTCTRL, r16

	//load period low byte then high byte
	ldi r16, LOW(PERIOD)		
	sts TCA0_SINGLE_PER, r16
	ldi r16, HIGH(PERIOD)
	sts TCA0_SINGLE_PER + 1, r16

	;set clock and start timer
	ldi r16, TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm
	sts TCA0_SINGLE_CTRLA, r16

	//configre VREF
	ldi r16, 0x03
	sts VREF_ADC0REF, r16

	//CONFIGURE PRESCALAR
	ldi r16, 0x0A
	sts ADC0_CTRLC, r16

	//SELECT ANALOG INPUT 
	ldi r16, 11
	sts ADC0_MUXPOS, r16

	//enable ADC 
	ldi r16, 0x01
	sts ADC0_CTRLA, r16
	
	//ENABLE INTERUPS
	sei
	
	//post display
	ldi r16, 0x00
	ldi r18, 0x04
	rcall post_display

	//start conversion
	ldi r16, 0x01
	sts  ADC0_COMMAND, r16

main_loop: 
	lds r16, ADC0_INTFLAGS
	sbrc r16, 0
	rcall conversion_done
	rjmp main_loop


;***************************************************************************
;* 
;* "conversion_done"
;*
;* Description:Takes result of conversion and performs arithmetic 
;* operations (multiplcation and division) in order to convert ADC output to 
;* be displayed on 7 segment display 
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.1
;* Last updated:			12/16/21
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r16, r18
;* High registers modified: r16, r18
;*
;* Parameters: r16 : initial value to show on startup 
;*			   r18 : holds size of array (loop controll variable)
;* Returns:	   void
;*
;* Notes: 
;* r16 is set to 0xFF after sequence to black display 
;***************************************************************************

conversion_done:
	lds r16, ADC0_RESL 
	lds r17, ADC0_RESH
	ldi r19, HIGH(2500)
	ldi r18, LOW(2500)
	rcall mpy16u

	lsr r21
	ror r20
	ror r19

	lsr r21
	ror r20
	ror r19

	lsr r21
	ror r20
	ror r19

	lsr r21
	ror r20
	ror r19

	ldi r22, Low(500)
	ldi r23, high(500)

	sub r19, r22
	sbc r20, r23

	mov r16, r19
	mov r17, r20


	rcall bin16_to_BCD
	rcall poll_digit_entry
	ldi r16, 0x01
	sts  ADC0_COMMAND, r16
	sts ADC0_INTFLAGS, r16
	ret
;***************************************************************************
//Toggles pin interupt service request for multiplex display on post
;***************************************************************************

toggle_post_ISR:
	push r16			
	in r16, CPU_SREG
	push r16
	push r17

	rcall multiplex_display

	ldi r16, TCA_SINGLE_OVF_bm	
	sts TCA0_SINGLE_INTFLAGS, r16

	pop r17				
	pop r16
	out CPU_SREG, r16
	pop r16

	reti

;***************************************************************************
;* 
;* "post_display"
;*
;* Description:Does status check on Segment display (Flashes for one second)
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.1
;* Last updated:			12/16/21
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r16, r18
;* High registers modified: r16, r18
;*
;* Parameters: r16 : initial value to show on startup 
;*			   r18 : holds size of array (loop controll variable)
;* Returns:	   void
;*
;* Notes: 
;* r16 is set to 0xFF after sequence to black display 
;***************************************************************************
post_display: 
	push r16
	push r18

	ldi XH, HIGH(led_display)
	ldi XL, LOW(led_display)

	rcall load_led_display

	ldi r16, 0xFF

	rcall load_led_display

	pop r18
	pop r16


;***************************************************************************
;* 
;* "load_led_display"
;*
;* Description:loads the contents of led_display with appropriate values to output to 7 segment display
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.1
;* Last updated:			12/16/21
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r16, r18
;* High registers modified: r16, r18
;*
;* Parameters: r16 : holds value to fill led_display array with
;*			   r17 : holds size of array (loop controll variable)
;* Returns:	   void
;*
;* Notes: 
;* subrouting uses loop controll variable in r17 to fill memory array 
;***************************************************************************
load_led_display:
	push r16
	push r18

	st X+, r16
	dec r18
	brne load_array
	
	rcall delay

	ldi r16, 0xFF
	breq load_array
	pop r18
	pop r16

	ret


;***************************************************************************
;* 
;* "delay"
;*
;* Description:loads the contents of led_display with appropriate values to output to 7 segment display
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.1
;* Last updated:			12/16/21
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r16, r17, r21
;* High registers modified: r16, r17, r21
;*
;* Parameters: r16 : loop controll variable
;*			   r17 : loop controll variable
;*			   r21 : loop controll variable

;* Returns:	   void
;*
;* Notes: 
;* Values are hardcoded for 40 MHz clock cycle of AVR128DB48 producing an 
;* approximate 1 second delay 
;***************************************************************************
delay:
	push r16
	push r17
	push r21

delay_1s:
	ldi r21, 25
	delay2:
		ldi r17, 200
		delay1:
			ldi r16, 100
			delay0:
				nop
				dec r16
			brne delay0
			nop
			dec r17
		brne delay1
	dec r21
	brne delay2

	pop r21
	pop r17
	pop r16
	      
    ret 

;***************************************************************************
;* "multiplex_display"
;*
;* Description:Updates a single digit of the display and increments the digit num to the digit position
;* to be displayed next (stored in digit_num)
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.1
;* Last updated:			12/16/21
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	
;* High registers modified: 
;*
;* Parameters: led_display : a four byte memory array that holds the segment values for each digit of the display
;*			   digit_num: a byte memory variable that has the least sig two bits holding the index of last digit displayed

;* Returns:	   Outputs segment pattern and turns ON digit driver for the next position in the display to be turned on
;*
;* Notes: 
;* Segments are controlled by VPORTD (dp, a through g), the digit drivers are controlled by VPORTA (PA7 - PA4, digit 0 - 3)
;***************************************************************************
multiplex_display: 
push_regs: 
	push r16
	push r17
	push r18

	ldi r16, 0xFF
	out VPORTD_OUT, r16
	in r16, VPORTA_OUT
	ori r16, 0xF0
	out VPORTA_OUT, r16
	ldi XH, HIGH(led_display)
	ldi XL, LOW(led_display)
	lds r16, digit_num
	inc r16
	andi r16, 0x03
	sts digit_num, r16
	add XL, r16
	brcc PC+2
	inc XH
	ld r17, X
	out VPORTD_OUT, r17
	in r17, VPORTA_OUT
	ldi r18, 0b10000000
digit_pos:
	cpi r16, 0
	breq digit_on
	lsr r18
	dec r16
	rjmp digit_pos
digit_on: 
	eor r17, r18
	out VPORTA_OUT, r17

pop_regs: 
	pop r18
	pop r17
	pop r16
	
	ret 
;***************************************************************************
;* "convert_bcd_entries"
;*
;* Description:converts results of ADC to hexidecimal, and then passes to look up table to display on LED display
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.1
;* Last updated:			12/16/21
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	
;* High registers modified: 
;*
;* Parameters: 

;* Returns:	   
;* Notes: 
;* 
;***************************************************************************
convert_bcd_entries:
push_registers: 
	push r16
    push r17 
	push r18
    push r19
    push r20
    push r21
	push r22

    ldi ZL, LOW(bcd_entries)
    ldi ZH, HIGH(bcd_entries)
    ldi YL, LOW(led_display)
    ldi YH, HIGH(led_display)
    
	mov r17, r22
	swap r17
	andi r17, 0x0F
	andi r22, 0x0F
	st z, r22
	std z+1, r17

	mov r17, r23
	swap r17
	andi r17, 0x0F
	andi r22, 0x0F
	std z+2, r22
	std z+3, r17
   
   	ldi r22, 0x04

convert_entries: 
	ld r18, Z+
    rcall hex_to_7seg
    st Y+, r18
	dec r22
	breq pop_registers
	rjmp convert_entries

    
pop_registers:
	pop r22
    pop r21
    pop r20
    pop r19
	pop r18
    pop r17
	pop r16
    ret

;***************************************************************************
;* 
;* "hex_to_7seg" - Hexadecimal to Seven Segment Conversion
;*
;* Description: Converts a right justified hexadecimal digit to the seven
;* segment pattern required to display it. Pattern is right justified a
;* through g. Pattern uses 0s to turn segments on ON.
;*
;* Author:			Ramy Abdulazziz
;* Version:			0.1						
;* Last updated:		101221
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:
;* Low registers modified:
;* High registers modified:
;*
;* Parameters: r18: hex digit to be converted
;* Returns: r18: seven segment pattern. 0 turns segment ON
;*
;* Notes: 
;*
;***************************************************************************
hex_to_7seg:
	push r18
    push ZL 
    push ZH
    push r16
    ldi ZH, HIGH(hextable * 2)  
    ldi ZL, LOW(hextable * 2)
    ldi r16, $00                
    andi r18, 0x0F                
    add ZL, r18
    adc ZH, r16
    lpm r18, Z
    pop r16
    pop ZH
    pop ZL    
	pop r18         
    ret

    ;Table of segment values to display digits 0 - F
    ;!!! seven values must be added
hextable: .db $01, $4F, $12, $06, $4C, $24, $20, $0F, $00, $4F, $04, $08, $60, $31, $42, $38





;***************************************************************************
;* 
;* "bin16_to_BCD" - 16-bit Binary to BCD Conversion
;*
;* Description: Converts a 16-bit unsigned binary number to a five digit
;* packed BCD number. Uses subroutine div16u from Atmel application note AVR200
;*
;* Author:					Ramy Abdulazziz
;* Version:					0.0
;* Last updated:			111320
;* Target:					
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r14, r15
;* High registers modified: r16, r17, r18, r19, r20, r22, r23, r24
;*
;* Parameters: r17:r16 16-bit unsigned right justified number to be converted.
;* Returns:		r24:r23:r22 five digit packed BCD result.
;*
;* Notes: 
;* Subroutine uses repeated division by 10 to perform conversion.
;***************************************************************************
bin16_to_BCD:
	ldi r19, 0			;high byte of divisor for div16u
	ldi r18, 10			;low byte of the divisor for div16u

	rcall div16u		;divide original binary number by 10
	mov r22, r14		;result is BCD digit 0 (least significant digit)
	rcall div16u		;divide result from first division by 10, gives digit 1 
	swap r14			;swap digit 1 for packing
	or r22, r14			;pack

	rcall div16u		;divide result from second division by 10, gives digit 2
	mov r23, r14		;place in r23
	rcall div16u		;divide result from third division by 10, gives digit 3 
	swap r14			;swap digit 3 for packing
	or r23, r14			;pack

	rcall div16u		;divide result from fourth division by 10, gives digit 4
	mov r24, r14		;place in r24

	ret


;Subroutine div16u is from Atmel application note AVR200

;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;*# "dd16uH:dd16uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:	clr	drem16uL	;clear remainder Low byte
	sub	drem16uH,drem16uH;clear remainder High byte and carry
	ldi	dcnt16u,17	;init loop counter
d16u_1:	rol	dd16uL		;shift left dividend
	rol	dd16uH
	dec	dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:	rol	drem16uL	;shift dividend into remainder
	rol	drem16uH
	sub	drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	drem16uL,dv16uL	;    restore remainder
	adc	drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1


;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret

