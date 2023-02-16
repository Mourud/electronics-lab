/*
 * PHYS319 Interrupt Example in C
 * Written by Ryan Wicks
 * Modified by A.K,
 */
#include <msp430.h>

int i = 0;
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
	P1DIR = 0b00000001;		  // set P1.0 pin for output rest including  P1.1 are inputs
	P4DIR = 0b10000000;		  // set P4.7 pin for output
	P1OUT = 0b00000011;		  // set Pin P1.0 to high and P1.1 to pullup
	P4OUT = 0b10000000;		  // set Pin P4.7  to high
	P1REN = 0b00000010;		  // enable pull up/down resistor on P1.1
	P1IE = 0b00000010;		  // Enable input at P1.1 as an interrupt
	// Part 4
	P1OUT &= ~0b00000001; // set Pin P1.0 to low
	P4OUT &= ~0b10000000; // set Pin P4.7  to low

	_BIS_SR(LPM4_bits + GIE); // Turn on interrupts and go into the lowest
							  // power mode (the program stops here)
							  // Notice the strange format of the function, it is an "intrinsic"
							  // ie. not part of C; it is specific to this microprocessor
}

void __attribute__((interrupt(PORT1_VECTOR))) PORT1_ISR(void) // Port 1 interrupt service routine
{															  // code of the interrupt routine goes here

	// 	//  	P1OUT			P2OUT
	// 	// R	0b xxxx xxx1	0b 0xxx xxxx
	// 	// G	0b xxxx xxx1	0b 1xxx xxxx
	// 	// RG	0b xxxx xxx1	0b 1xxx xxxx
	// 	// ~R~G	0b xxxx xxx0	0b 0xxx xxxx

		_delay_cycles(200000); // delay for 200000 cycles
		switch (i%4)
		{
		case 0:
			P1OUT |= 0b00000001; // set Pin P1.0 to high
			break;
		case 1:
			P1OUT &= ~0b00000001; // set Pin P1.0 to low
			P4OUT |= 0b10000000; // set Pin P4.7  to high
			break;
		case 2:
			P1OUT |= 0b00000001; // set Pin P1.0 to high
			break;
		case 3:
			P1OUT &= ~0b00000001; // set Pin P1.0 to low
			P4OUT &= ~0b10000000; // set Pin P4.7  to low
			break;
		}
		i++;
		P1IFG &= ~0b00000010;
		
}