//*1. Device starts up in LPM3 + blinking LED to indicate device is alive
 //    + Upon first button press, device transitions to application mode
 // 2. Application Mode
 //    + Continuously sample ADC Temp Sensor channel
 //
 //    + Transmit temperature value via TimerA UART to PC


#include  "msp430.h"

#define     LED1                  BIT0                         //for P1.0
#define     LED2                  BIT7                         //for P4.7

#define     BUTTON                BIT1

#define     TXD                   BIT4                      // TXD on P4.4
#define     RXD                   BIT5                      // RXD on P4.5

#define     PreAppMode            0
#define     RunningMode           1


#define CALADC12_15V_30C  *((unsigned int *)0x1A1A)   // Temperature Sensor Calibration-30 C
                                                      //See device datasheet for TLV table memory mapping
#define CALADC12_15V_85C  *((unsigned int *)0x1A1C)   // Temperature Sensor Calibration-85 C


unsigned int TXByte;
volatile unsigned int Mode;

void InitializeButton(void);
void PreApplicationMode(void);

void main(void)
{
  long tempMeasured;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  /* next  line to use internal calibrated 1.048MHz clock: */

  TA0CTL = TASSEL_2 + MC_1 + TAIE +ID_3;            // Timer A control set to SMCLK, 1MHz and count up mode MC_1



  InitializeButton();


  P1DIR |= LED1;
  P4DIR |= LED2;
  P1OUT &= ~LED1;     //LEDs off
  P4OUT &= ~LED2;
  P4DIR |= TXD;
  P4OUT |= TXD;


  Mode = PreAppMode;
  __enable_interrupt();      // Enable interrupts.
  PreApplicationMode();          // Blinks LEDs, waits for button press

  /* Configure ADC Temp Sensor Channel */

  REFCTL0 &= ~REFMSTR;  // Reset REFMSTR to hand over control to ADC12_A ref control registers

  ADC12CTL0 |= ADC12SHT03 + ADC12ON + ADC12REFON;  // Sampling time 32 cycles, ADC12 on and reference voltage for Temperature sensor is on
  ADC12CTL1 |= ADC12SHP;                     // sampling timer
  ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_10;  // Select internal Vref and A10 = temp sense i/p
  ADC12CTL0 |= ADC12ENC ;                     // enable conversion

   __delay_cycles(1000);                     // Wait for ADC Ref to settle


  /* Configure hardware UART */
  UCA1CTL1 = UCSWRST; //Recommended to place USCI in reset first
  P4SEL |= BIT4 + BIT5;
  UCA1CTL1 |= UCSSEL_2; // Use SMCLK
  UCA1BR0 = 109; // Set baud rate to 9600 with 1.048MHz clock (Data Sheet 36.3.13)
  UCA1BR1 = 0; // Set baud rate to 9600 with 1.048MHz clock
  UCA1MCTL = UCBRS_2; // Modulation UCBRSx = 2
  UCA1CTL1 &= ~UCSWRST; // Initialize USCI state machine
  /* if we were going to receive, we would also:
     IE2 |= UCA1RXIE; // Enable USCI_A1 RX interrupt
  */

  /* Main Application Loop */
  while(1)
  {
    ADC12CTL0 |= ADC12SC;        // Sampling and conversion start


    while (! (UCA1IFG & UCTXIFG)); // wait for TX buffer to be ready for new data
    tempMeasured = ADC12MEM0;
    tempMeasured = ((tempMeasured - CALADC12_15V_30C) * (85 - 30)) / (CALADC12_15V_85C - CALADC12_15V_30C) + 30.0;


    TXByte = tempMeasured;
    UCA1TXBUF = TXByte;//Transmit TXByte;



    P1OUT ^= LED1;  // toggle the light every time we make a measurement.
    P4OUT &= ~LED2;// make sure that green LED is off

    // go to sleep, wait till timer expires to do another measurement.

   _delay_cycles(500000);

  }
}

void PreApplicationMode(void)
{
  P1DIR |= LED1;
  P4DIR |= LED2;
  P1OUT |= LED1;                 // To enable the LED toggling effect
  P4OUT &= ~LED2;

  while (Mode == PreAppMode)  {
    P1OUT ^= LED1;
    P4OUT ^= LED2;                      // toggle the two lights.
    _delay_cycles (500000);             //This function introduces 0.5 s delay

                              }


}

void InitializeButton(void)                 // Configure Push Button
{
  P1DIR &= ~BUTTON;
  P1OUT |= BUTTON;
  P1REN |= BUTTON;
  P1IES |= BUTTON;
  P1IFG &= ~BUTTON;
  P1IE |= BUTTON;
}

/* *************************************************************
 * Port Interrupt for Button Press
 * 1. During standby mode: to enter application mode
 *
 * *********************************************************** */

void __attribute__ ((interrupt(PORT1_VECTOR))) PORT1_ISR(void) // Port 1 interrupt service routine
 {                                                    //code of the interrupt routine goes here
    Mode = RunningMode;

    P1IE &= ~BUTTON;         // Disable port 1 interrupts
    P1IFG &= ~0b00000010;        // Clear P1.1 IFG.If you don't, it just happens again.
 }
