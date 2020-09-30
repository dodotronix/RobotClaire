	.file	"ATmega8_control_board.c"
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__SREG__ = 0x3f
__tmp_reg__ = 0
__zero_reg__ = 1
	.text
.global	I2Cinitialize
	.type	I2Cinitialize, @function
I2Cinitialize:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(80)
	out 0x2,r24
	ldi r24,lo8(68)
	out 0x36,r24
	ret
	.size	I2Cinitialize, .-I2Cinitialize
.global	I2CReceiveTransmitt
	.type	I2CReceiveTransmitt, @function
I2CReceiveTransmitt:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r18,lo8(-56)
	ldi r19,0
.L4:
	in __tmp_reg__,0x36
	sbrs __tmp_reg__,7
	rjmp .L4
	in r24,0x1
	andi r24,lo8(-8)
	cpi r24,lo8(-88)
	brne .L5
	lds r24,Value
	lds r25,Value+1
	cpi r24,-1
	cpc r25,__zero_reg__
	breq .+2
	brge .L6
	out 0x3,r24
	in r24,0x36
	andi r24,lo8(-65)
	rjmp .L12
.L5:
	in r24,0x1
	andi r24,lo8(-8)
	cpi r24,lo8(-120)
	brne .L6
	in r25,0x3
	lds r24,IR_sensor0
	cpse r25,r24
	rjmp .L7
	sts Value+1,r19
	sts Value,r18
.L7:
	in r24,0x36
	ori r24,lo8(-64)
.L12:
	out 0x36,r24
.L6:
	in r24,0x1
	andi r24,lo8(-8)
	cpi r24,lo8(-64)
	brne .L8
	in r24,0x36
	ori r24,lo8(-64)
	rjmp .L11
.L8:
	in r24,0x1
	andi r24,lo8(-8)
	cpi r24,lo8(96)
	brne .L4
	in r24,0x36
	andi r24,lo8(-65)
.L11:
	out 0x36,r24
	rjmp .L4
	.size	I2CReceiveTransmitt, .-I2CReceiveTransmitt
	.section	.text.startup,"ax",@progbits
.global	main
	.type	main, @function
main:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(-128)
	out 0x11,r24
	ldi r24,lo8(32)
	out 0x17,r24
	rcall I2Cinitialize
	rcall I2CReceiveTransmitt
	.size	main, .-main
	.comm	Value,2,1
	.comm	adresa,1,1
.global	IR_sensor0
	.data
	.type	IR_sensor0, @object
	.size	IR_sensor0, 1
IR_sensor0:
	.byte	-96
	.ident	"GCC: (GNU) 4.9.2"
.global __do_copy_data
.global __do_clear_bss
