#include <msp430.h> 


/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

        P6DIR = 0b00001111;
        P4DIR = 0b00000011;
        P2DIR = 0b00000001;
        P4OUT = 0b00000011;


        // make an array with 1,4,4,6
        unsigned int arr[4] = {	0b00000001, //1
                               	0b00000100, //4
                            	0b00000100, //4
								0b00000110};//6

         int i =0;
         for (i = 0; i < 4; i++) {
            //GET NUMBER TO SET FROM ARR
            P6OUT = arr[i];


            //STROBE
            P2OUT = 0b00000000;
            P2OUT = 0b00000001;

            //SELECT NEXT DISPLAY FOR NEXT LOOP
            P4OUT -= 0b00000001;
         }

	

	return 0;
}

