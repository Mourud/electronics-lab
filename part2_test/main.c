#include <stdio.h>
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 


int main(void){
    unsigned int P1OUT, P2OUT, P4OUT, P6OUT;
    P4OUT = 0b00000011;
    int arr[4] = {1,4,4,6};
    for (int i = 0; i < 4; i++) {
	    unsigned int P1OUT = arr[i]; // SHOULD be P6?
        P2OUT = 0b00000000;
		P2OUT = 0b00000001;
		//DEBUG: print P1OUT and P4OUT
		printf("P1OUT "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(P1OUT));
        printf("\t");
        printf("P4OUT "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(P4OUT));
        printf("\n");

		P4OUT -= 0b00000001;
	}
}