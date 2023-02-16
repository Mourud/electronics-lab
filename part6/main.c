#include <msp430.h>
void main(void)
{
  WDTCTL = WDTPW | WDTHOLD; // Stop WDT
  P1DIR |= BIT2;            // Output on Pin 1.2
  P1SEL |= BIT2;            // Pin 1.2 selected as PWM
  TA0CCR0 = 1024;            // PWM period PWM period (512/1.048)
                            // microseconds
  TA0CCR1 = 512;             // PWM duty cycle
  TA0CCTL1 = OUTMOD_7;      // TA0CCR1 reset/set-high voltage
  // below count, low voltage when past

  TA0CTL = TASSEL_2 + MC_1 + TAIE + ID_0;
  // Timer A control set to SMCLK, 1MHz
  // and count up mode MC_1
  _bis_SR_register(LPM0_bits + GIE); // Enter Low power mode 0 and enable inrerrupts
}
