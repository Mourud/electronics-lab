#!/usr/bin/python2.7
import serial # for serial port
import numpy as np # for arrays, numerical processing

#needs: python2, pyserial, numpy, 
#0) flash the serial temperature measurement program into the msp430
#1) start this program
#2) press the button the Launchpad to enter 'applcation mode'
#3) warm the chip (eg with a light bulb or your fingers)
#4) when you've seen enough, press the reset button on the launchpad
#5) exit the program by pressing 'q' or clicking on the x

#define the serial port. Pick one:
#port = "/dev/ttyACM0"  #for Linux
port = "/dev/tty.usbmodem1203"
with serial.Serial(port,9600,timeout = 0.050) as ser:
    print(ser.name)
    while(1): #loop forever
        data = ser.read(1) # look for a character from serial port - will wait for up to 50ms (specified above in timeout)
        if len(data) > 0: #was there a byte to read?
            print(ord(data))