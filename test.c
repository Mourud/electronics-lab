#include <msp430.h>

/**
 * main.c
 */

//  Starts at 0
//  Waits for Push
//  Increments 2s
//  Outputs on LED as binary
//  Outputs on MD as decimal
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    P1DIR = 0b00000000; // Set P1.1 as input
    P1OUT = 0b00000010; // Set P1.1 to pullip
    P1REN = 0b00000010; // enable pull up/down resistor on P1.1
    P1IE = 0b00000010;	// Enable input at P1.1 as an interrupt
	P1IES = 0b00000010; // Set interrupt on falling edge (button 


    P4DIR = 0b00000011; // P4.0-4.1 set as outputs
    P4OUT = 0b00000000; // P4.0 P4.1 is set to the latch of MD

    P2DIR = 0b00000001; // P2.0 connected to strobe


    P6DIR = 0b00001111; // Sets P6.0 - P6.3 to be outputs
    P6OUT = 0b00000000; // Sets initial value to 0

    return 0;
}

void __attribute__((interrupt(PORT1_VECTOR))) PORT1_ISR(void) // Port 1 
{                                                              
    while (1)
    {
        // STROBE
        P2OUT = 0b00000000;
        P2OUT = 0b00000001;
        _delay_cycles(2000000); // Delay cycle is set to 2s
        P6OUT += 1;             // Increments the value by 1
        // Add safeguards for overflow
        P6OUT &= 0b00000010; // Only allows P6.0 - P6.3 to have HIGH values
    }
}