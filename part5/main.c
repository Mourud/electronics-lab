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

  P1DIR |= 0x01;                            // set pin P1.0 as output
  P4DIR |= 0x80;                            // set pin P4.7 as output
  P1DIR |= 0x04;                            // set pin P1.2 as output


  while (1)
  {

    ADC12CTL0 |= ADC12SC;                   // Start sampling
    while (ADC12CTL1 & ADC12BUSY);          //while bit ADC12BUSY in register ADC12CTL1 is high wait

    //ADC12MEM0 >= 3072 is the value of the ADC when the input voltage is >2.4V
    //
    if(ADC12MEM0>=2978){                    //If V > 2.4V
      P1OUT |= BIT0;                        //Turn on LED red
      P1OUT &= ~BIT2;                       //Turn off LED yellow
      P4OUT &= ~BIT7;                       //Turn off LED green
    }
    else if(ADC12MEM0>=620){                //If V > 0.5V and V < 2.4V
      P1OUT &= ~BIT0;                       //Turn off LED red
      P1OUT |= BIT2;                     //Turn on LED yellow
      P4OUT &= ~BIT7;                       //Turn off LED green
    }
    else{                                   //If V < 0.5V
      P1OUT &= ~BIT0;                       //Turn off LED red
      P1OUT &= ~BIT2;                    //Turn off LED yellow
      P4OUT |= BIT7;                        //Turn on LED green
      
    }
  }
}
