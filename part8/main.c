#include <msp430f5529.h>

#define TRIG BIT1
#define ECHO BIT2

unsigned int TXByte;
unsigned int time;

long calcDistance(unsigned int time);

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD; // Stop WDT

  P8DIR = TRIG; // Set P8.1(Trig) to output direction and P8.2(Echo) to input direction
  P8REN = ECHO;
  P8OUT = 0;          // Set Trig pin to be low
  UCA1CTL1 = UCSWRST; // Reset the UART module
  P4DIR = BIT4;
  P4SEL |= BIT4 + BIT5; // Configure P4.4 and P4.5 for UART operation
  UCA1CTL1 |= UCSSEL_2; // Select SMCLK as the clock source for the UART
  UCA1BR0 = 109;        // Set the baud rate to 9600 (based on 1.048 MHz clock)
  UCA1BR1 = 0;
  UCA1MCTL = UCBRS_2;              // Set the modulation UCBRSx to 2
  UCA1CTL1 &= ~UCSWRST;            // Enable the UART module
  TA0CTL = TASSEL_2 + MC_0 + ID_0; // Timer A control set to SMCLK, stop count MC_0 and clear counter, no division – //keep 1.048MHz,

  while (1)
  {
    P8OUT |= TRIG;
    __delay_cycles(20);
    P8OUT &= ~TRIG;
    // wait for the echo pin to go high
    while ((P8IN & ECHO) == 0);

    TA0CTL |= MC_2; // Start timer
    TA0R = 0;       // Clear timer
    
    // wait for the echo pin to go low
    while ((P8IN & ECHO) == 4);
    // read the counter and find the time in us
    time = TA0R * 1 / 1.048;
    // wait to send the next signal
    while (!(UCA1IFG & UCTXIFG));
    long distance = calcDistance(time);
    // send distance to the computer using UART
    TXByte = distance;
    UCA1TXBUF = TXByte; // Transmit TXByte;

    __delay_cycles(100000);
  }
}

long calcDistance(unsigned int time)
{
  // distance = time * speed of sound / 2
  // distance is in cm
  long distance = time * 0.034 / 2;
  return distance;
}
