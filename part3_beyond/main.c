/*
* PHYS319 timing Example in C
* Written by Ryan Wicks
* Modified by A.K,
*/
#include <msp430.h>
void main(void) {
    WDTCTL = WDTPW + WDTHOLD; //Stop WDT
    P1DIR =0b00000001; //Set P1.0 to output;
	P6DIR =0b00000000;
    P4DIR =0b10000000; //Set P4.7 to output;
    P1OUT = 0b00000000; //Set the output Pin P1.0 to low
    // P4OUT = 0b10000000; //Set the output Pin P4.7 to high
    P4OUT = 0b00000000; //Set the output Pin P4.7 to low

	P6OUT = 0b01111111;
	P6REN = 0b01111111;
    while (1) { // Loop forever
        //_delay_cycles (500000); //This function introduces 0.5 s delay 
        //Changed delay to be half of the original
		//loop over P6 values:

		int i =0;
		unsigned int d;
		for (i = 0; i < 7; i++)
		{
			d = P6IN & 0b00000001<<i;
			blink_select = d & 0b00000001;
            P1OUT &= blink_select
			_delay_cycles(100000);
			P1OUT &= ~0b00000001;
			_delay_cycles(250000);
		}

        // P1OUT = P1OUT ^ 0b00000001; //bitwise xor the output with 00000001 
        // P4OUT = P4OUT ^ 0b10000000; //bitwise xor the output with 10000000 
    }
}
