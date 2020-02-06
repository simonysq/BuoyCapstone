#Main program for our Capstone Project
#Sponsor: Penn State ARL 1
#Project Title: UUV Mast, Fall 2019
#Class: EE403W Section 2
#Professor: Mark Bregar

#Overview
#The goal here is to create an algorithm that turn on the appropriate thrusters
#when our cylinder moves from the original location.

import time
import board
import busio
import adafruit_gps
import serial
import math
import RPi.GPIO as IO

##########################
#VARIABLES
BACKWARD = 0.13
FORWARD = 0.17
STOP = 0.15
FREQ = 100
DUTY_CYCLE = 15

#THRUSTER PINS
NORTH = 26
SOUTH = 21
EAST = 20
WEST = 16

#OTHER PINS
TIMER = 19

#ARRAYS
PINS = [NORTH, SOUTH, WEST, EAST, TIMER]

#####################################
#INITIALIZATION
#sets up the board to send PWM signals to
#all the four thrusters

IO.setmode(IO.BCM)

for pin in PINS:
    IO.setup(pin, IO.OUT)

pwm_n = IO.PWM(NORTH, FREQ)
pwm_n.start(DUTY_CYCLE)

pwm_s = IO.PWM(SOUTH, FREQ)
pwm_s.start(DUTY_CYCLE)

pwm_e = IO.PWM(EAST, FREQ)
pwm_e.start(DUTY_CYCLE)

pwm_w = IO.PWM(WEST, FREQ)
pwm_w.start(DUTY_CYCLE)

##########################################

#ARRAYS
PWMs = [pwm_n, pwm_s, pwm_e, pwm_w]


#INPUT: none
#OUTPUT: none
#Desc: Timer for 15 seconds that turns pin
#      TIMER high and low every second.
def run_timer_LED():
    
    for x in range(15, 0, -1):
        IO.output(TIMER, True)
        #print("%d seconds left.\n" %x)
        time.sleep(1)
        IO.output(TIMER, False)
        time.sleep(1)

#INPUT: lat (float), long (float)
#OUTPUT: float, float
#DESC: returns speed for latitude and longitude thrusters
def calculate_speeds(lat, long):
    
    angle=math.degrees(math.atan(abs(lat)/abs(long)))
    return angle*(0.0002/90), (90-angle)*(0.0002/90)
    
#INPUT: pwm
#OUTPUT: none
#DESC: Stops the thruster for input pwm
def stop_thruster(pwm):
    
    pwm.ChangeFrequency(FREQ)
    pwm.ChangeDutyCycle(DUTY_CYCLE)

#INPUT: none
#OUTPUT: none
#DESC: Stops all four thrusters
def stop_all_thrusters():
    
    for pwm in PWMs:
        stop_thruster(pwm)

#INPUT: pwm, frequency (float), dc (float)
#OUTPUT: none
#DESC: Changes the speed of the thruster controlled by input pwm
def run_thruster(pwm, frequency, dc):
    
    pwm.ChangeFrequency(frequency)
    pwm.ChangeDutyCycle(dc)

#INPUT: none
#OUTPUT: gps
#DESC: Initilizes the GPS module      
def setup_GPS():

    uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=10)
    gps = adafruit_gps.GPS(uart, debug=False)     # Use UART/pyserial
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')
    return gps
    

def main():
    
    gps = setup_GPS() #sets up the GPS
    last_print = time.monotonic() #captures the time
    
    #wait till GPS finds a fix
    while not gps.has_fix:
        # Try again if we don't have a fix yet.
        gps.update()
        #print('Waiting for fix...')
        
    run_timer_LED() #wait for 15 seconds, once fix has been found
    
    #grabs original location
    gps.update()
    lat_OG = gps.latitude
    long_OG = gps.longitude
     
    #MAIN LOOP
    while True:
        try:
            gps.update()
            
            #calculate change in lat and long
            long_change = gps.longitude - long_OG
            lat_change = gps.latitude - lat_OG
            
            current = time.monotonic() #captures current time
            if current - last_print >= 0.1: #compares so algorithm runs every 0.1s
                last_print = current

                #case1: NO CHANGE ---> Turn all thrusters off
                if(long_change == 0 and lat_change == 0):
                    stop_all_thrusters()
                
                #case2: CHANGE IN LATITUDE
                elif(long_change == 0):
                    
                    #turn off North and South Thrusters
                    stop_thruster(pwm_n)
                    stop_thruster(pwm_s)
                    
                    #run West and East Thrusters
                    if(lat_change >0):
                        #if position is above original position
                        run_thruster(pwm_w, 1, BACKWARD)
                        run_thruster(pwm_e, 1, BACKWARD)
                        
                    elif(lat_change <0):
                        #if position is below original position
                        run_thruster(pwm_w, 1, FORWARD)
                        run_thruster(pwm_e, 1, FORWARD)
                
                #case3: CHANGE IN LONGTITUDE
                elif(lat_change == 0):
                    
                    #turn off WEST and EAST Thrusters
                    stop_thruster(pwm_w)
                    stop_thruster(pwm_e)
                    
                    #run NORTH and SOUTH Thrusters
                    if(long_change >0):
                        #if poisiton is left of original position
                        run_thruster(pwm_n, 1, FORWARD)
                        run_thruster(pwm_s, 1, FORWARD)
                    elif(long_change <0):
                        #if position is right of original poisition
                        run_thruster(pwm_n, 1, BACKWARD)
                        run_thruster(pwm_s, 1, BACKWARD)
                    
                #case4: CHANGE IN BOTH 
                elif(long_change != 0 and lat_change != 0):
                    
                    NS, WE = calculate_speeds(lat_change, long_change)
                    
                    #if position is above and right to the original position
                    #(1st quadrant)
                    if(long_change >0 and lat_change >0):
                        run_thruster(pwm_w, 1, STOP-WE)
                        run_thruster(pwm_e, 1, STOP-WE)
                        
                        run_thruster(pwm_n, 1, STOP+NS)
                        run_thruster(pwm_s, 1, STOP+NS)
                    
                    #if position is below and right to the original position
                    #3rd Quadrant
                    elif(long_change >0 and lat_change <0):
                        
                        run_thruster(pwm_n, 1, STOP+NS)
                        run_thruster(pwm_s, 1, STOP+NS)
                        
                        run_thruster(pwm_w, 1, STOP+WE)
                        run_thruster(pwm_e, 1, STOP+WE)
                    
                    #if position is above and left to the original position
                    #2nd Quadrant
                    elif(long_change <0 and lat_change >0):
                        
                        run_thruster(pwm_w, 1, STOP-WE)
                        run_thruster(pwm_e, 1, STOP-WE)
                        
                        run_thruster(pwm_n, 1, STOP-NS)
                        run_thruster(pwm_s, 1, STOP-NS)
                    
                    #if position is below and left to the original position
                    #4th Quadrant
                    elif(long_change <0 and lat_change <0):
                        
                        run_thruster(pwm_w, 1, STOP+WE)
                        run_thruster(pwm_e, 1, STOP+WE)
                        
                        run_thruster(pwm_n, 1, STOP-NS)
                        run_thruster(pwm_s, 1, STOP-NS)
           
        except KeyboardInterrupt:
            break
      
      
    IO.cleanup()
