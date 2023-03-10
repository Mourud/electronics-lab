
# Heart Monitor Using MAX30102 and MSP430
# - Written by Adam Darcie (2022)
# - You may need to start this BEFORE flashing the MSP
# - There is no pre-application mode - when this program starts, it is waiting for data


# Use python 3.7 or 3.8 (Anaconda 3 and Spider)
# starting the program
#!/usr/bin/python3.7
import serial  # for serial port - MAKE SURE THIS IS FROM PYSERIAL NOT SERIAL PACKAGE

#Should all come automatically with Anaconda
import matplotlib.pyplot as plt
import time
import numpy as np
import csv

############# USER INPUT ##############
#MSP/MAX settings
NSTARTBYTES = 2#2  # indicate number of bytes used to indicate start of transmission
SAMPLE_RATE = 50  # Hz - make sure this corresponds to MAX sample rate
SKIP_DATA = 14 #should match SKIP_DATA on MSP - change here and on MSP if there is bad data at the start of each transmission
BLOCK_DATA_LENGTH = 96# should match BLOCK_DATA_LENGTH on MSP

#plotting/display settings
create_plot = True  # set true to create updating plot
spo2_mode=0#set to 1 for spo2 mode, set 0 for HR mode (change both lines in MSP first)
plot_last=(BLOCK_DATA_LENGTH-SKIP_DATA)*2  # plot only the last plot_last points
plot_nan=False#decide whether to end each transmission with NaN (shows break in plot between transmissions, but is not handled well in excel)
n_avg=1#rolling average between n_avg adjacent points (useful for noisy data with wide peaks)

# define the serial port. Pick one:
# port = "/dev/ttyACM0"  #for Linux
port = "COM5"  # For Windows "COM_"
# port = "/dev/tty.uart-XXXX" #For Mac?

# give folder and file name for where to save processed data
# MAKE SURE FILE IS NOT OPEN WHILE RUNNING
# DATA WILL OVERWRITE ANYTHING IN THIS FILE
csv_savename = "C:\\Users\\kotlicki\\Documents\\319\\2023\\AK programs\\heartdata.csv"
#######################################

# attempt to automatically pop out plot - if fails, do so in console
try:  
    import IPython
    shell = IPython.get_ipython()
    shell.enable_matplotlib(gui='qt')
except:
    print("Need to enter '%matplo tlib qt' manually in console.")
    pass


# rolling average funciton
def rolling_avg(datalist):
    avglist = []
    avgnum = n_avg  # average over avgnum points
    for i in range(len(datalist)-avgnum):
        elem = 0
        for j in range(avgnum):
            elem += datalist[i+j]

        avglist.append(elem/avgnum)
    return avglist


#lists to store data
fullreddata = []
fullIRdata = []
timeseries = []
times = []
lasttime = 0

#initialize figure
if create_plot:
    plt.figure()

# write commands in keyboard interrupt:
# - continuously collect data into "fulldata" until user presses ctrl-c
# - then saves "fulldata" to a csv file
try:
    #open serial communication
    with serial.Serial(port, 19200, timeout=0.1) as ser:
        print(ser.name)
        print("Flushing serial...")
        ser.flushInput()
        ser.flushOutput()
        print("Flushed")
        start_time = time.time()
        while(1):  # loop forever to collect data continuously
            rawdata =  ser.readline()
            # print(rawdata)
            data = list(rawdata)#converts string of bytes to list of ints
            # print(data)

            #if there aren't at least enough start bytes, don't bother continuing - go back and look for more data
            if len(data) > (NSTARTBYTES):
                trimdata = np.array(data[NSTARTBYTES:])#trim away start bytes
                # print("======= New Transmission =====")
                # print("# BYTES received: "+str(len(trimdata)))
                
                if len(trimdata) != ((BLOCK_DATA_LENGTH-SKIP_DATA)*2):  # make sure there are enough bytes
                    print("Incorrect number of Bytes received.")
                else:
                    # get current time and save in list
                    current_time = time.time()-start_time
                    times.append(current_time)

                    # convert [high low high low... ] byte sequence to 16-bit int
                    # even (high) byte * 256 + odd (low) byte
                    highbytes = trimdata[0::2]
                    lowbytes = trimdata[1::2]
                    convdata = np.add(highbytes*256, lowbytes)
                    
                    # convdata=convdata[convdata>0]

                    if spo2_mode:
                        # interpret alternating bytes as red vs IR data
                        # - change relative LED current to verify which is which
                        reddata = convdata[1::2]
                        IRdata = convdata[0::2]
                    else:
                        #if HR mode just collect IR data (all data is IR data)
                        IRdata=convdata

                    # apply rolling average and extend existing row
                    if spo2_mode:
                        avgreddata = rolling_avg(reddata)
                        fullreddata.extend(avgreddata)
                        if plot_nan:
                            fullreddata.append(float("nan"))# add nan to each list to indicate end of each transmission
                        # print("Number of data points (red): "+str(len(reddata)))
                    avgIRdata = rolling_avg(IRdata)
                    fullIRdata.extend(avgIRdata)
                    if plot_nan:
                        fullIRdata.append(float("nan"))# add nan to each list to indicate end of each transmission
                    # print("Number of data points (IR): "+str(len(IRdata)))


                    # interpolate across time based on sample rate to get time series
                    if len(times) == 1:
                        add_timeseries = np.linspace(times[len(times)-1], times[len(times)-1]+len(avgIRdata)/SAMPLE_RATE, len(avgIRdata), axis=0)
                        dt = times[0]
                    else:
                        add_timeseries = np.linspace(times[len(times)-1], len(avgIRdata)/SAMPLE_RATE+times[len(times)-1], len(avgIRdata), axis=0)
                        dt = times[len(times)-1]-times[len(times)-2]
                    timeseries.extend(add_timeseries)
                    
                    #give the time per cycle and compare to the time given by sample rate and data length
                    # print("dt: "+str(dt)+"s"+" (should be ~" +str(BLOCK_DATA_LENGTH/SAMPLE_RATE/(spo2_mode+1))+" s)")
                    
                    #append nan to show breaks in plot where no data is recorded
                    if plot_nan:
                        timeseries.append(float("nan"))
                        
                    
                    #do plotting
                    if create_plot:
                        plt.pause(0.01)# wait for plot to update (INCREASE IF PLOT ISN'T UPDATING PROPERLY)
                        plt.clf()  # clear figure
                        if len(timeseries) < plot_last:
                            if spo2_mode:
                                plt.plot(timeseries, fullreddata,label="Red")
                            plt.plot(timeseries, fullIRdata, label="IR")
                        else:
                            pltind = len(timeseries)-plot_last
                            if spo2_mode:
                                plt.plot(timeseries[pltind:],fullreddata[pltind:], label="Red")
                            plt.plot(timeseries[pltind:],fullIRdata[pltind:], label="IR")
                        plt.ylabel("Power (arb)")
                        plt.xlabel("Time (s)")
                        plt.legend()
                        plt.grid()

#on ctrl-c (in console), save to CSV in specified location
except KeyboardInterrupt:
    print("Collection stopped - saving to CSV...")
    startcsv = time.time()
    with open(csv_savename, "w", newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if spo2_mode:
            rows = zip(timeseries, fullIRdata,fullreddata)
        else:
            rows = zip(timeseries, fullIRdata)
        for row in rows:
            writer.writerow(row)
    endcsv = time.time()
    print("Data saved to "+csv_savename)
    print("Took "+str(endcsv-startcsv)+" s")
