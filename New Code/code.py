'''
This is the software-based simulation of the UUV's code.
NEED TO WORK ON: Allow the UUV to simulate itself moving towards the HOME coordinates to gauge the effectiveness
of the overall code. That is, once the script generates a random coordinate system that is [x, y] off of [0, 0],
simulate the UUV moving back to [0, 0].
As of current, the UUV is able to adjust its speed. Need to implement either functions or variables that will
keep track of the UUV's current position relative to its HOME coordinates.

EX:
LONGTITUDE:

0 = x +- current_speed * time
'''

import math
import time
import random

###############################################
############## For Classes ONLY ###############

# Sets up a class responsible for controlling each individual thruster's speed
class Thruster:
    def __init__(self, name, speed):
        self.name = name
        self.speed = speed

    def get_speed(self):
        return self.speed

    def set_speed(self, speed):
        self.speed = speed

    def get_name(self):
        return self.name

##############################################
################# Variables ##################
# Defines the maximum speed that the thrusters can go at and what the STOP signal is
MAX_FORWARD_SPEED   = 190
NEUTRAL             = 150
MAX_BACKWARD_SPEED  = 110

# Splits up the 4 thrusters' adjustment speeds into 2 sets: LONG (NS) and LAT (WE)
# ADJUSTMENT is added to NEUTRAL, which is what causes the thrusters to spin either foward or backwards
LONG_ADJUSTMENT     = 0
LAT_ADJUSTMENT      = 0

# Initializes adjustment_time, which is used to delay changes in thruster speeds
long_adjust_time    = 0
lat_adjust_time     = 0

# Defaults scale to 1
long_scale          = 1
lat_scale           = 1

##############################################
################## Objects ###################

# Sets up the four thrusters into instances
NORTH = Thruster("NORTH", NEUTRAL)
SOUTH = Thruster("SOUTH", NEUTRAL)
EAST = Thruster("EAST", NEUTRAL)
WEST = Thruster("WEST", NEUTRAL)

##############################################
################## Arrays ####################
PINS = [NORTH, SOUTH, WEST, EAST]

##############################################
##### Simulation Functions and Variables #####

# Functions
def random_change():
    # generates and returns a value between -1 to 1, step value of 0.01
    return random.randrange(-100, 100, 1) / 100

# Variables
# Sets up the home location to 0, 0 -- corresponds to a XY coordinate system
HOME_LAT = 0
HOME_LONG = 0

# Calls random_change for each one, generating a random LAT/LONG to start off
LAT = random_change()
LONG = random_change()

##############################################
########## Functions and Main Loop ###########

# Hardware initialization of thrusters here

'''def setup_GPS():
    uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=10)
    gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')
    return gps'''

'''def setup_IMU():
    uart = serial.Serial("/dev/serial2", baudrate=9600, timeout=10)
    # TODO: IMU code'''

def gps_fix():
    return print("Fix detected, continuing...\n")

# Remove for hardware integration
def gps_update():
    global LONG, LAT
    LONG = random_change()
    LAT = random_change()
    return

# Stops the thrusters given by the array PINS
def stop_thrusters(PINS):
    for PIN in PINS:
        PIN.set_speed(NEUTRAL)
        if (PIN == NORTH) or (PIN == SOUTH):
            long_scale = 1
        if (PIN == EAST) or (PIN == WEST):
            lat_scale = 1
        print("Stopping Thruster {}".format(PIN.get_name()))
    time.sleep(0.01)

# Sets the relevant thrusters to a speed defined by DUTYCYCLE
def run_thrusters(PINS, DUTYCYCLE):
    for PIN in PINS:
        PIN.set_speed(DUTYCYCLE)
        print("Setting thruster {0} to {1}".format(PIN.get_name(), DUTYCYCLE))
    print("")
    time.sleep(0.1)

# Tolerance compares the magnitudes of the LONG/LAT values with a fixed value,
# which corresponds to the +- 10ft specification
def tolerance(LONG, LAT):
    tolerance = 0.01
    tolerance_list = [False] * 2

    if abs(LONG) > 0.01:
        tolerance_list[0] = True
    if abs(LAT) > 0.01:
        tolerance_list[1] = True

    message = "Tolerance Check\n{0:<20s}{1:<.4f}\n{2:<20s}{3:<}\n{4:<20s}{5:<}\n".format(
        "Tolerance Value:", tolerance, "Longitude:", str(tolerance_list[0]), "Latitude:", str(tolerance_list[1])
    )
    print(message)
    return tolerance_list

# Can be optimized
# Takes in thrusters that need to be adjusted
# t = time of last adjustment
# scale = how much should the adjustment to speed be
# coord defines if the change is either a positive or negative

def adjust_speed(PINS, t, scale, coord):
    adjustment = 10 * scale

    if (time.monotonic() - t > 1) or scale == 1:
        if (10 * scale) >= 41:
            scale = scale
        else:
            scale += 1
            adjustment = 10 * (scale - 1)
            t = time.monotonic()

            if coord > 0:
                run_thrusters(PINS, NEUTRAL - adjustment)
            else:
                run_thrusters(PINS, NEUTRAL + adjustment)

        return [t, scale, adjustment]

    else:
        return [t, scale, adjustment]


def main():
    # Comment out/remove once simulated movement has been implemented
    time_since_GPS_change = time.monotonic()

    # Calls for GPS location
    print("Waiting for a fix...\n")
    gps_fix()

    # Updates current position
    gps_update()

    # Compares change from HOME location [0, 0]
    long_change = HOME_LONG - LONG
    lat_change = HOME_LAT - LAT

    # Main Loop
    while(1):
        global adjustment_time, LAT_ADJUSTMENT, LONG_ADJUSTMENT, lat_adjust_time, long_adjust_time
        global lat_scale, long_scale

        last_runtime = time.monotonic()
        try:
            current_time = time.monotonic()

            # Update and compare coordinates
            #gps_update()
            long_change = HOME_LONG - LONG
            lat_change = HOME_LAT - LAT

            #Delay update for a maximum of 0.1s
            time.sleep(5 - (current_time - last_runtime))

            last_runtime = current_time

            # Calls tolerance to see if out of bounds
            case = tolerance(long_change, lat_change)

            # Changes set location -- remove/comment out if necessary
            print(time.monotonic() - time_since_GPS_change)
            if time.monotonic() - time_since_GPS_change >= 20:
                time_since_GPS_change = time.monotonic()
                gps_update()
                long_scale = 0
                lat_scale = 0
                print(long_change)
                print(lat_change)

            # Responsible for LONG
            # Currently barebones
            if case[0]:
                if long_adjust_time == 0:
                    long_adjust_time = time.monotonic()
                change = adjust_speed([NORTH, SOUTH], long_adjust_time, long_scale, long_change)
                long_adjust_time = change[0]
                long_scale = change[1]
                LONG_ADJUSTMENT = change[2]

            else:
                stop_thrusters([NORTH, SOUTH])

            # Responsible for LAT
            # Currently barebones
            if case[1]:
                if lat_adjust_time == 0:
                    lat_adjust_time = time.monotonic()
                adjust_speed([WEST, EAST], lat_adjust_time, lat_scale, lat_change)
                lat_adjust_time = change[0]
                lat_scale = change[1]
                LAT_ADJUSTMENT = change[2]

            else:
                stop_thrusters([WEST, EAST])

        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()
