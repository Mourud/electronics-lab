/*
* PHYS319 timing Example in C
* Written by Ryan Wicks
* Modified by A.K,
*/
#include <msp430.h>
void main(void) {
    WDTCTL = WDTPW + WDTHOLD; //Stop WDT
    P1DIR =0b00000001; //Set P1.0 to output;
    P4DIR =0b10000000; //Set P4.7 to output;
    P1OUT = 0b00000000; //Set the output Pin P1.0 to low
    // P4OUT = 0b10000000; //Set the output Pin P4.7 to high
    P4OUT = 0b00000000; //Set the output Pin P4.7 to low
    while (1) { // Loop forever
        //_delay_cycles (500000); //This function introduces 0.5 s delay 
        _delay_cycles (250000); //Changed delay to be half of the original
        P1OUT = P1OUT ^ 0b00000001; //bitwise xor the output with 00000001 
        P4OUT = P4OUT ^ 0b10000000; //bitwise xor the output with 10000000 
    }
}