#include <msp430f5529.h>
#include<stdio.h>

int main(void)
{
  
  WDTCTL = WDTPW + WDTHOLD;
  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // sampling timer
  ADC12CTL0 |= ADC12ENC;                    // ADC enable
  P6SEL |= 0x01;                            // P6.0 allow ADC on pin 6.0
  ADC12MCTL0 = ADC12INCH_0;                 // selects which input results are 
                                            // stored in memory ADC12MEM0. Input
                                            // one is selected on reset so this line is not needed

  P1DIR |= BIT2;                            // Output on Pin 1.2
  P1SEL |= BIT2;                            // Pin 1.2 selected as PWM
  TA0CCR0 = 50;                             // PWN period = 50/1MHz = 50us
  
                            // microseconds
  TA0CCTL1 = OUTMOD_7;      // TA0CCR1 reset/set-high voltage
  // below count, low voltage when past

  TA0CTL = TASSEL_2 + MC_1 + TAIE + ID_0;
  // Timer A control set to SMCLK, 1MHz
  // and count up mode MC_1
  // _bis_SR_register(LPM0_bits + GIE); // Enter Low power mode 0 and enable inrerrupts

  while (1)
  {

    ADC12CTL0 |= ADC12SC;                   // Start sampling
    while (ADC12CTL1 & ADC12BUSY);          //while bit ADC12BUSY in register ADC12CTL1 is high wait
    char c = ADC12MEM0 * TA0CCR0/4095;
    TA0CCR1 = c;

  }
}
