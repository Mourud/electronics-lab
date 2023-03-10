/*
Heart Monitor Using MAX30102
Written by Shengman Zhang and Colby Delisle (2021)
 - modified by Adam Darcie and AK (2022)

Circuit:
-connect sensor directly to MSP using the following connections:

GND   -   GND    use the pins below P4.1
VCC   -   VIN
P4.2  -   SCL
P4.1  -   SDA


Starting Program:
-use with python-serial-plot-heartbeat.py
    -tested with python 3.8
-if MSP will not flash, run python program first and leave running

Operation:
-red LED on MSP should blink several times to confirm proper initialization
-LEDs on MAX should be bright-dim-bright to confirm i2c communication (RED led only seen in spo2 mode)
-red LED on MSP will turn on while waiting for data to fill the FIFO
-once FIFO is read, 16-bit data is transmitted via serial in series of (high-low-high-low...) byte transmissions
-16-bit data samples alternate (IR-red-IR-red...) if spo2 mode is used, only IR in HR mode
-interpret data as arbitrary photodetector reading

 */




#include "msp430.h"
#include "stdint.h"
#include "intrinsics.h"

// I2C communication MSP430
#define SDA_PIN BIT0 // SDA (serial data) is P4.1
#define SCL_PIN BIT1 // SCL (serial clock) is P4.2


// MAX30102 definitions (referring to Manual)

// Define Addresses
#define     LED1                  BIT0           //for P1.0 red
#define     LED2                  BIT7           //for P4.7 green

#define     BUTTON                BIT1            //For P1.1

#define     TXD                   BIT4             // TXD on P4.4
#define     RXD                   BIT5             // RXD on P4.5

#define MAX30102_SLAVE_ID 0x57 //slave address for MAX30102 is /0x57
#define INT_STATUS_1_REG_ADD 0x00
#define INT_ENABLE_1_REG_ADD 0x02
#define FIFO_WR_PTR_REG_ADD 0x04
#define FIFO_OVR_CNT_REG_ADD 0x05
#define FIFO_RD_PTR_REG_ADD 0x06
#define FIFO_DATA_PTR_REG_ADD 0x07
#define FIFO_CONFIG_REG_ADD 0x08
#define MODE_CONFIG_REG_ADD 0x09
#define SPO2_CONFIG_REG_ADD 0x0A
#define LED1_PULSE_AMP_REG_ADD 0x0D// **LED1 and LED2 are reversed in manual**
#define LED2_PULSE_AMP_REG_ADD 0x0C// - testing has revealed these are the correct addresses

// Communication options
#define A_FULL BIT7
#define RESET BIT6
#define FIFO_SIZE 32 //DON'T CHANGE
#define NUMBER_OF_TIMES_READ_FIFO 3 //from 3
#define BLOCK_DATA_LENGTH 96 //changed from 96
#define SKIP_DATA 14 //skip this number of data points from heart monitor buffer
#define UCB1CTL1_INIT_VALUE 0x80 //sets the I2C SMCLK mode and receive condition.

// Defining functions
void Write_Single_Byte(uint8_t, uint8_t);
uint8_t Read_Single_Byte(uint8_t RegAddress);
void READ_FIFO(void);
void Blink_Target_LED(void);

// Initializing MAX30102 Sensor
uint8_t Spo2_Config_Reg_Data = 0; // Data for Spo2 Config register
uint8_t Led1_Pulse_Amp_Reg_Data = 0; // RED LED Current register
uint8_t FIFO_Config = 0; // FIFO Configuration
uint8_t FIFO_WR_Ptr_Reg_Data = 0; // Data for write pointer register
uint8_t FIFO_Ovr_Cnt_Reg_Data = 0; // over counter register
uint8_t FIFO_RD_Ptr_Reg_Data = 0; // Read pointer register
uint8_t Mode_Config_Reg_Data = 0; // Mode control register
uint8_t MAX_Int_1_Reg_Data = 0; // Interrupt enable register
uint16_t HR_Data[BLOCK_DATA_LENGTH] = {0}; // An array with samples of HR data
uint16_t Sample_Counter = 0; // Sample Counter

int main( void )
{
  P1DIR |= LED1;
  P4DIR |= LED2;
  P1OUT &= ~LED1;     //LEDs off
  P4OUT &= ~LED2;
  P4DIR |= TXD;
  P4OUT |= TXD;

    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    uint8_t k=0; // loop counter

    // LED on the MSP430 is used to indicate loading config data and also showing data collection
    // Blink LED on MSP430 to make sure it is working

    Blink_Target_LED();


    P4SEL  |= 0b00000110; //select P4.1 and P4.2 to enable RXD and TXD for UART

    Blink_Target_LED();

    // Initialize MSP430 I2C port
    UCB1CTL1 = UCSWRST; // Reset USCI to enable I2c configuration
    UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC; // set MSP430 as Master, synchronous mode
    UCB1CTL1 = UCSSEL_2 + UCSWRST; // Use SMCLK, keep USCI reset
    UCB1BR0 = 20; // I2C Data transmission rate 50 kHz
    UCB1BR1 = 0;
    UCB1STAT = 0; // Clear all STAT bits
    UCB1IE = 0; // clear I2C interrupt enable registers
    UCB1IFG = 0; // Clear all interrupt flag bits (including I2C related)
    UCB1I2CSA = MAX30102_SLAVE_ID; // Set slave address
    UCB1CTL1 &= ~UCSWRST; // Clear SW reset, resume operation

    Blink_Target_LED();

    // ============= Configure Serial Transmission ================
    // Configure hardware UART (python transmission)
    uint8_t TXByte;
  TA0CTL = TASSEL_2 + MC_1 + TAIE +ID_3;  // Timer A control set to SMCLK, //1MHz and count up mode MC_1
  UCA1CTL1 = UCSWRST; //Recommended to place USCI in reset first
  P4SEL |= BIT4 + BIT5;
  UCA1CTL1 |= UCSSEL_2; // Use SMCLK
  UCA1BR0 = 54; // Set baud rate to 19200 with 1.048MHz clock //(Data Sheet 36.3.13)
  UCA1BR1 = 0;
  UCA1MCTL = UCBRS_2; // Modulation UCBRSx = 2
  UCA1CTL1 &= ~UCSWRST; // Initialize USCI state machine
  /* if we were going to receive, we would also:
     UCA1IE2 |= UCA1RXIE; // Enable UCA1IE2 RX interrupt*/

    //Test Read_Single_Byte
 k=Read_Single_Byte(0xFF); // This is to test that Read_Single_Byte() function works. The readback value should be 0x15 (21) in  UCB1RXBUF
  //Led1_Pulse_Amp_Reg_Data = 0x0F; // = 3 mA current into RED LED
 // Write_Single_Byte(LED1_PULSE_AMP_REG_ADD, 0x0F);





    // ============= Configure MAX30102 sensor ==============

    //Mode Configuration (0x09)
    Mode_Config_Reg_Data = 0b00000010; // Initialize Mode control register to (last 2 bits) HR Mode=10, spo2 mode=11
    Write_Single_Byte(MODE_CONFIG_REG_ADD, Mode_Config_Reg_Data);

    //SpO2 Configuration (0x0A)
    Spo2_Config_Reg_Data = 0b00100001;// first bit reserved, SPO2_ADC_RGE = 4096 (01),
    //sample rate = 50Hz (000) or 100Hz (001), LED pulse width control = 118 (adc res 16) (01)
    Write_Single_Byte(SPO2_CONFIG_REG_ADD, Spo2_Config_Reg_Data);

    Blink_Target_LED();

    //Test sensor LEDs and set pulse amplitude
    Led1_Pulse_Amp_Reg_Data = 0x0F; // = 3 mA current into RED LED
    Write_Single_Byte(LED1_PULSE_AMP_REG_ADD, Led1_Pulse_Amp_Reg_Data);
    Led1_Pulse_Amp_Reg_Data = 0x0F; // = 3 mA current into IRRED LED
    Write_Single_Byte(LED2_PULSE_AMP_REG_ADD, Led1_Pulse_Amp_Reg_Data);
    __delay_cycles(1000000); // wait for 1 sec

    Led1_Pulse_Amp_Reg_Data = 0x03; // 0.6 mA current into RED LED
    Write_Single_Byte(LED1_PULSE_AMP_REG_ADD, Led1_Pulse_Amp_Reg_Data);
    Led1_Pulse_Amp_Reg_Data = 0x03; // 0.6 current into IRRED LED
    Write_Single_Byte(LED2_PULSE_AMP_REG_ADD, Led1_Pulse_Amp_Reg_Data);
    __delay_cycles(1000000); // wait for 1 sec


    Led1_Pulse_Amp_Reg_Data = 0x3F; // = 12.6 mA current into RED LED
    Write_Single_Byte(LED1_PULSE_AMP_REG_ADD, Led1_Pulse_Amp_Reg_Data);
    Led1_Pulse_Amp_Reg_Data = 0x2F; // ~4-5 mA current into IRRED LED
    Write_Single_Byte(LED2_PULSE_AMP_REG_ADD, Led1_Pulse_Amp_Reg_Data);
    __delay_cycles(1000000); // wait for 1 sec


    Blink_Target_LED();

    // ================ Configure FIFO =====================


    // FIFO Configuration (0x08)
    FIFO_Config = 0b00000001; // FIFO config Reg, with sample averaging=0 (bits 8-6 000)
    // FIFO_ROLL_OVER_EN (bit 5), FIFO_A_FULL (bits 4-0)
    Write_Single_Byte(FIFO_CONFIG_REG_ADD, FIFO_Config);
    k=Read_Single_Byte(FIFO_CONFIG_REG_ADD);

    // FIFO Write Pointer (0x04) - points to location where next sample is written
    FIFO_WR_Ptr_Reg_Data = 0; // Initialize write pointer register
    Write_Single_Byte(FIFO_WR_PTR_REG_ADD, FIFO_WR_Ptr_Reg_Data);

    // Overflow Counter (0x05) - counts number of samples lost due to full FIFO
    FIFO_Ovr_Cnt_Reg_Data = 0; // Initialize over counter register
    Write_Single_Byte(FIFO_OVR_CNT_REG_ADD, FIFO_Ovr_Cnt_Reg_Data);

    // FIFO Read Pointer (0x06) - points to the location from where the processor gets the next sample from the FIFO through the
    // I2C interface
    FIFO_RD_Ptr_Reg_Data = 0; // Initialize Read pointer register
    Write_Single_Byte(FIFO_RD_PTR_REG_ADD, FIFO_RD_Ptr_Reg_Data);

    // Interrupt enable 1 (0x02)/
    MAX_Int_1_Reg_Data = 0b10000000; // Initialize Interrupt enable register to generate interrupt when FIFO is full (128)
    // Bit 7 enables A_FULL interrupt
    Write_Single_Byte(INT_ENABLE_1_REG_ADD, MAX_Int_1_Reg_Data);
    k = Read_Single_Byte(INT_STATUS_1_REG_ADD); // read the interrupt status register




    uint8_t highByte;
    uint8_t lowByte;

    // Continuously read data from MAX and transmit over serial
    while (1)
    {

        // Read 32 samples from MAX31020 FIFO and store in HR_Data
        FIFO_WR_Ptr_Reg_Data = 0; // Initialize write pointer register
        Write_Single_Byte(FIFO_WR_PTR_REG_ADD, FIFO_WR_Ptr_Reg_Data);

        FIFO_Ovr_Cnt_Reg_Data = 0; // Initialize over counter register
        Write_Single_Byte(FIFO_OVR_CNT_REG_ADD, FIFO_Ovr_Cnt_Reg_Data);

        FIFO_RD_Ptr_Reg_Data = 0; // Initialize Read pointer register
        Write_Single_Byte(FIFO_RD_PTR_REG_ADD, FIFO_RD_Ptr_Reg_Data);

        Mode_Config_Reg_Data = 0b00000010; // Initialize Mode control register to (last 2 bits) HR Mode=10, spo2 mode=11
        Write_Single_Byte(MODE_CONFIG_REG_ADD, Mode_Config_Reg_Data);

        k = Read_Single_Byte(INT_STATUS_1_REG_ADD); // read the status register






        Sample_Counter = 0;
        for (k=0; k<NUMBER_OF_TIMES_READ_FIFO; k++)//read FIFO predetermined amount of times
        {
            READ_FIFO(); // Read and Save data in Memory
        }



        // ================ SERIAL TRANSMISSION ==================

        //send series of bytes indicating start of transmission (1,1)
        TXByte=(uint8_t) 0b00000001;
        while (! (UCA1IFG & UCTXIFG)); // wait for TX buffer to be ready for new data
        UCA1TXBUF = TXByte;
        while (! (UCA1IFG & UCTXIFG)); // wait for TX buffer to be ready for new data
        UCA1TXBUF = TXByte;



        //send each byte from HR_data over serial
        int datind;
        for (datind=SKIP_DATA; datind<BLOCK_DATA_LENGTH;datind++){

            // python doesn't accept 16-bit serial data natively, so send two 8-bit segments (high then low)

//            send byte 1 (high)
            highByte=(HR_Data[datind] >> 8) & 0xFF;
            if (highByte==10){//make sure reading is not 10 (new line character in Python)
                highByte=11;
            }

            TXByte=highByte;
            while (! (UCA1IFG & UCTXIFG)); // wait for TX buffer to be ready for new data
            UCA1TXBUF = TXByte;

//
//            //send byte 2 (low)
            lowByte = (HR_Data[datind] >> 0) & 0xFF;
            if (lowByte==10){//make sure reading is not 10 (new line character in Python)
                lowByte=11;
            }
            TXByte=lowByte;
            while (! (UCA1IFG & UCTXIFG)); // wait for TX buffer to be ready for new data
            UCA1TXBUF = TXByte;
            __delay_cycles(1000);

        }

    } // end of while(1)
   // return 0;
} // main function ends here


// Function to write single byte of data in MAX30120
void Write_Single_Byte (uint8_t RegAddress, uint8_t RegValue)
{
    while( UCB1CTL1 & UCTXSTP ); // if the stop bit from previous reading is not cleared, then wait here

        //UCB1CTL1 = UCB1CTL1_INIT_VALUE; // Initialize UCB1CTL1
        UCB1CTL1 |= UCTR; // Set the I2C in transmit mode.
        UCB1CTL1 |= UCTXSTT; // Set the start bit
        // Check whether UCTXIFG is set which means UCB1TXBUF is ready to accept the data
    while ((UCB1IFG & UCTXIFG) == 0);


    UCB1TXBUF = RegAddress; // Send the MAX30102 Register Address
    // Check whether UCTXIFG is set which means UCB1TXBUF is ready to accept the new data
    while ((UCB1IFG & UCTXIFG) == 0)
    {
        if (UCB1IFG & UCNACKIFG) // If there is no response from slave UCNACKIFG = 1, (error report)
        {
            // DO SOMETHING HERE if there is no response (optional)
        }
    }

    UCB1TXBUF = RegValue; // Send the contents to be loaded into the MAX30102 register
    // Check whether UCTXIFG is set which means UCB1TXBUF is empty again and then set the stop bit.
    while ((UCB1IFG & UCTXIFG) == 0);

        UCB1CTL1 |= UCTXSTP; // Send the stop bit

    // Wait until stop bit is cleared which means that the stop bit has been sent
    while (!(UCB1CTL1 & UCTXSTP) == 0);

    return;
}

// Function to Read single byte of data from MAX30120
uint8_t Read_Single_Byte(uint8_t RegAddress)
{
    volatile uint8_t Reg_data=0;
    UCB1IFG=0;
    UCB1CTL1 |= UCTR; // Set the I2C in transmit mode.
    UCB1CTL1 |= UCTXSTT; // Set the start bit
    // Check whether UCBxTXIFG is set which means UCB1TXBUF is ready to accept the data
    while ((UCB1IFG & UCTXIFG) == 0);

        UCB1TXBUF = RegAddress; // Send the MAX30102 Register Address
        // Check whether UCBxTXIFG is set which means UCB1TXBUF is ready to accept the new data
        while ((UCB1IFG & UCTXIFG) == 0);
    {
        if (UCB1IFG & UCNACKIFG) // If there is no response from slave UCNACKIFG = 1,(error report)
    {
    // DO SOMETHING HERE if there is no response
    }
    }
    // Set the I2C in repeat start mode
    UCB1CTL1 &= ~UCTR; // Set the I2C in receive mode
    UCB1CTL1 |= UCTXSTT; // Set the start bit
    while (UCB1CTL1 & UCTXSTT); // wait for UCTXSTT = 0
    // If there is no response UCNACKIFG = 1
    UCB1CTL1 |= UCTXSTP; // first send stop bit
    // Check until the data is received
    while ((UCB1IFG & UCRXIFG) == 0);
    {
    }
    Reg_data = UCB1RXBUF;
    return(Reg_data);
}




// Function to Read 32 sample data from FIFO in MAX30120
void READ_FIFO(void)
{
    volatile uint8_t Int_Status_1_Reg_Data = 0;
    volatile uint8_t BYTE1 = 0;
    volatile uint8_t BYTE2 = 0;
    volatile uint8_t BYTE3 = 0;
    volatile uint8_t sample_int_count=0;
    uint8_t k2 = 0;
    uint8_t k3 = 0;

    while(UCB1CTL1 & UCTXSTP);

    P1OUT |= BIT0; // Turn ON LED on MSP430 to start reading FIFO

    // Check to see whether FIFO is full or not.
    while ( (Int_Status_1_Reg_Data & A_FULL) == 0 )
    {
        // Get value of A_FULL in Interrupt status 1 (0x00)
        // - A_FULL goes high when there are 32-N samples in FIFO
        // - N is determined by FIFO_A_FULL (default 0)
        Int_Status_1_Reg_Data = Read_Single_Byte(INT_STATUS_1_REG_ADD);
    }

    //count each new sample in FIFO until it is full
//    while (sample_int_count<FIFO_SIZE){
//        sample_int_count += ~(Read_Single_Byte(INT_STATUS_1_REG_ADD) & BIT6);
//    }

    P1OUT &= ~BIT0; // Turn OFF LED on MSP430 when HR_Data has been updated

    // Read the data from FIFO in repeat start mode
    //UCB1CTL1 = UCB1CTL1_INIT_VALUE; // Initialize UCB1CTL1
    UCB1CTL1 |= UCTR; // Set the I2C in transmit mode.
    UCB1CTL1 |= UCTXSTT; // Set the start bit


    // Check whether UCB1TXIFG is set which means UCB1TXBUF is ready to accept the data
    while ((UCB1IFG & UCTXIFG) == 0);

    UCB1TXBUF = FIFO_DATA_PTR_REG_ADD; // Send the MAX30102 Register Address
    while ((UCB1IFG & UCTXIFG) == 0);
        {
        if (UCB1IFG & UCNACKIFG){ // If there is no response UCNACKIFG = 1
                                }
        }

    // Read the FIFO data. FIFO depth is 32 samples. Each sample has 3 bytes.
    // The useful data is saved from BIT17 down to BIT2 (16 bits resolution)
    UCB1CTL1 &= ~UCTR; // Set the I2C in receive mode
    UCB1CTL1 |= UCTXSTT; // Send start bit and read control byte
    while (UCB1CTL1 & UCTXSTT); // wait for UCTXSTT = 0


    // read all bytes from FIFO and append to HR_Data
    for (k2=0; k2 <= 3*FIFO_SIZE; k2++)
    {

        // Set the start bit. This goes to Repeat Start mode
        // Check until the data is received
        while ((UCB1IFG & UCRXIFG) == 0);


        if (k3 == 0) // save as first byte
        {
            BYTE1 = UCB1RXBUF;
            ++k3;
        }
        else if (k3 == 1) // save as second byte
        {
            BYTE2 = UCB1RXBUF;
            ++k3;
        }
        else if (k3 == 2) // otherwise it is the third byte
        {
            BYTE3 = UCB1RXBUF;
            k3 = 0;

//            refer to MAX manual for triplet readout procedure
//             - read first Byte and shift to bits 14,15
            HR_Data[Sample_Counter] = ( (uint8_t) BYTE1 ) << 14;
            // - read second Byte and shift to bits 6-13
            HR_Data[Sample_Counter] |= ( (uint8_t) BYTE2 ) << 6;
            // - read third Byte and shift to bits 0-5 (16-bit readout starts at bit 2)
            HR_Data[Sample_Counter] |= ( (uint8_t) BYTE3 ) >> 2;

            Sample_Counter++;
        }
    } // End of for (k2=0; k2 < 3*FIFO_SIZE; k2++)


    UCB1CTL1 |= UCTXSTP; // Send the stop bit
    while (!(UCB1IFG & UCRXIFG) == 0);
    HR_Data[Sample_Counter] = UCB1RXBUF;// Read the last byte
    while( UCB1CTL1 & UCTXSTP ); // wait for UCTXSTT = 0

    return;
}

// Function to Blink the LED
void Blink_Target_LED(void)
{
    P1OUT |= BIT0; // Turn ON LED on MSP430
    __delay_cycles(200000); // wait for 0.2 sec
    P1OUT &= ~BIT0; // Turn OFF LED on MSP430
    __delay_cycles(200000); // wait for 0.2 sec
}
