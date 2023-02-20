#include <msp430f5529.h>
#include <stdio.h>

int main(void)
{

    WDTCTL = WDTPW + WDTHOLD;
    P1DIR |= BIT2; // Output on Pin 1.2
    P1SEL |= BIT2; // Pin 1.2 selected as PWM

    // microseconds
    TA0CCTL1 = OUTMOD_7; // TA0CCR1 reset/set-high voltage
    // below count, low voltage when past

    TA0CTL = TASSEL_2 + MC_1 + TAIE + ID_0;
    // Timer A control set to SMCLK, 1MHz
    // and count up mode MC_1

	// Notes of Baba Black Sheep
    int notes[] = {261, 261, 392, 392, 440, 440, 392, 349, 349, 330, 330, 294, 294, 261, 392, 392, 349, 349, 330, 330, 294, 392, 392, 349, 349, 330, 330, 294};

    while (1)
    {

    int i = 0;
    for (i = 0; i < 28; i++){
        int period = 1000000 / notes[i];
        TA0CCR0 = period;
        TA0CCR1 = period / 2;
        __delay_cycles(500000);

    }
    }
}
