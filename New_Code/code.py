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

class Buoy:
    def __init__(self, lat_coord, long_coord, lat_speed, long_speed, orientation):
        self.lat_coord = lat_coord
        self.long_coord = long_coord
        self.lat_speed = lat_speed
        self.long_speed = long_speed
        self.orientation = orientation

    def set_lat_coord(self, coord):
        self.lat_coord += coord

    def get_lat_coord(self):
        return self.lat_coord

    def set_long_coord(self, coord):
        self.long_coord += coord

    def get_long_coord(self):
        return self.long_coord

    def set_lat_speed(self, speed):
        self.lat_speed = speed

    def get_lat_speed(self):
        return self.lat_speed

    def set_long_speed(self, speed):
        self.long_speed = speed

    def get_long_speed(self):
        return self.long_speed

    def set_orientation(self, rotateAngle):
    	self.orientation = (self.orientation + rotateAngle) % 360 

    def get_orientation(self):
    	return self.orientation

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
# Actual hardware has max forward pulsewidth of 1900us and reverse of 1100us and a stop of 1500us
# In simulation, we map 1900us to 4 and 1100us to -4.
MAX_FORWARD_SPEED   = 4
NEUTRAL             = 0
MAX_BACKWARD_SPEED  = -4

# Splits up the 4 thrusters' adjustment speeds into 2 sets: LONG (NS) and LAT (WE)
# ADJUSTMENT is added to NEUTRAL, which is what causes the thrusters to spin either forward or backwards
long_adjustment     = 0
lat_adjustment      = 0

# Stores the UUV's axis' signs. NORTH/EAST is positive. WEST/SOUTH is negative.
# These values only changes once sign_int's value is different than sign_current
# These values are set prior to any speed adjustment
long_sign_int   = 0
lat_sign_int    = 0

# These values are set as the system is running and is compared against sign_int
long_sign_current   = 0
lat_sign_current    = 0

# Initializes adjustment_time, which is used to delay changes in thruster speeds
long_adjust_time    = 0
lat_adjust_time     = 0

# Defaults scale to 1
long_scale          = 1
lat_scale           = 1

# Previous Location
long_old = 0
lat_old = 0
##############################################
################## Objects ###################

# Sets up the four thrusters into instances: thrusters are all off. 
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
def random_change(step_value):
    # generates and returns a value between |100/step_value| with a step of 1/step_value
    return random.randrange(-100, 100, 1) / step_value

'''def update_position():
    adjusted_lat_speed = (long_scale * 10) - wave_lat_current
    adjusted_long_speed = (lat_scale * 10) - wave_long_current

    print(adjusted_lat_speed)
    print(adjusted_long_speed)'''


# Variables
# Sets up the home location to 0, 0 -- corresponds to a XY coordinate system
HOME_LAT = 0
HOME_LONG = 0

# Calls random_change for each one, generating a random LAT/LONG to start off
LAT = random_change(5)
LONG = random_change(5)

# Wave Current (from -4 to +4 m/s)
wave_lat_current    = random_change(33)
wave_long_current   = random_change(33)

# Orientation: attach sensor at front of UUV (eg. north thruster), select random angle from 0 to 360 degrees
# to be oriented front of UUV must face north, (90 degrees out of 360 degrees)
ORIENT = random.randrange(360)

# UUV speed
UUV_long_speed      = 0
UUV_lat_speed       = 0

# Initializes the Buoy object
UUV = Buoy(LAT, LONG, wave_lat_current, wave_long_current, ORIENT)

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
    LONG = random_change(5)
    LAT = random_change(5)
    return

# Remove for hardware intergration, since there will be some compass/imu present for orientation. 
def orientation_update():
	global ORIENT
	ORIENT = random.randrange(360)

# Stops the thrusters given by the array PINS
def stop_thrusters(PINS):
    for PIN in PINS:
        PIN.set_speed(NEUTRAL)
        if (PIN == NORTH) or (PIN == SOUTH):
            long_scale = 1
        if (PIN == EAST) or (PIN == WEST):
            lat_scale = 1
        '''print("Stopping Thruster {}".format(PIN.get_name()))'''
    time.sleep(0.01)

# Sets the relevant thrusters to a speed defined by DUTYCYCLE
def run_thrusters(PINS, DUTYCYCLE):
    for PIN in PINS:
        PIN.set_speed(DUTYCYCLE)
        '''print("Setting thruster {0} to {1}".format(PIN.get_name(), DUTYCYCLE))
    print("")'''
    time.sleep(0.1)

# Tolerance compares the magnitudes of the LONG/LAT values with a fixed value,
# which corresponds to the +- 10ft specification
def tolerance(coord):
    tolerance = 3

    if abs(coord) > tolerance:
        return True
    else:
        return False
    '''message = "Tolerance Check\n{0:<20s}{1:<.4f}\n{2:<20s}{3:<}\n{4:<20s}{5:<}\n".format(
        "Tolerance Value:", tolerance, "Longitude:", str(tolerance_list[0]), "Latitude:", str(tolerance_list[1])
    )
    print(message)'''

# Can be optimized
# Takes in thrusters that need to be adjusted
# t = time of last adjustment
# scale = how much should the adjustment to speed be
# coord defines if the change is either a positive or negative

def adjust_speed(PINS, t, scale, coord, speed):
    global lat_old, long_old
    adjustment = scale
    if (time.monotonic() - t > 1) or scale == 1:
        if scale >= 4000:
            scale = scale
        elif not tolerance(coord):
            return [t, scale, adjustment]
        elif abs(speed) >= 1:
            scale *= 2
        else:
            scale += 1

        adjustment = 4 * math.exp(scale/10 - 5)/(math.exp(scale/10 - 5) + 1)

        t = time.monotonic()

        if coord > 0:
            run_thrusters(PINS, NEUTRAL - adjustment)
        else:
            run_thrusters(PINS, NEUTRAL + adjustment)

        return [t, scale, adjustment]

    else:
        return [t, scale, adjustment]

#for thruster to orient the UUV, plug return value into adjust speeds to adjust north thruster speed to 0.23 (about 5 degree/iteration) 
def rotateForwOrBack(PIN, angle): 
	if angle < 90 or angle > 270:
		run_thrusters(PIN, 0.23)
	else:
		run_thrusters(PIN, -0.23)

def coord_sign(coord):
    if coord >= 0:
        return 1
    else:
        return -1

def main():
    # Comment out/remove once simulated movement has been implemented
    time_since_GPS_change = time.monotonic()

    # Calls for GPS location
    print("Waiting for a fix...\n")
    gps_fix()

    # Updates current position
    gps_update()

    # Update Orientation. 
    orientation_update()

    # Compares change from HOME location [0, 0]
    lat_sign_int = coord_sign(LAT)
    long_sign_int = coord_sign(LONG)

    #update the UUV values, notice that UUV is moving at speed of currents. 
    UUV.set_lat_coord(LAT)
    UUV.set_long_coord(LONG)
    UUV.set_lat_speed(wave_lat_current)
    UUV.set_long_speed(wave_long_current)
    UUV.set_orientation(ORIENT)

    # Main Loop
    while(1):
        global adjustment_time, lat_adjustment, long_adjustment, lat_adjust_time, long_adjust_time
        global lat_scale, long_scale, orientation_scale, long_sign_current, lat_sign_current

        last_runtime = time.monotonic()
        try:
            current_time = time.monotonic()

            # UCompare coordinates
            long_sign_current = coord_sign(UUV.get_long_coord())
            lat_sign_current = coord_sign(UUV.get_lat_coord())

            #Delay update for a maximum of 0.1s
            time.sleep(1 - (current_time - last_runtime))
            wait = time.monotonic() - last_runtime

            #set current speed and location of UUV
            UUV.set_long_speed(NORTH.get_speed() - wave_long_current)
            UUV.set_lat_speed(WEST.get_speed() - wave_lat_current)
            UUV.set_long_coord(UUV.get_long_speed() * wait)
            UUV.set_lat_coord(UUV.get_lat_speed() * wait)

            #set new orientation of UUV:
            # w = v / r: v is in m/s, r = 5 cm so 0.05m 
            omega = NORTH.get_speed() / 0.05 
            # angle change = omega * change in time:
            changeAngle = omega * wait
            UUV.set_orientation(changeAngle)

            print("{0:14s}{1:>5f}\n{2:14s}{3:>5f}\n{4:14s}{5:>5f}\n".format("Lat Post:", UUV.get_lat_coord(),
                                                          "Long Post:", UUV.get_long_coord(), "Orientation:", UUV.get_orientation()))

            #print(wave_lat_current, wave_long_current)

            '''print("{0:12s}{1:>5f}\n{2:12s}{3:>5f}\n".format("Long Speed:", UUV.get_long_speed(),
                                                          "Lat Speed:", UUV.get_lat_speed()))'''

            last_runtime = current_time

            # Changes set location -- remove/comment out if necessary
            #print(time.monotonic() - time_since_GPS_change)
            '''if time.monotonic() - time_since_GPS_change >= 20:
                time_since_GPS_change = time.monotonic()
                gps_update()
                long_scale = 0
                lat_scale = 0
                print(long_change)
                print(lat_change)'''

            #Before moving, must ensure orientation of UUV is correct: approximate 90 degrees (+- 3 degrees)
            tempOrientation = UUV.get_orientation()
            if tempOrientation > 93 or tempOrientation < 87:
            	rotateForwOrBack([NORTH], tempOrientation)

            #only if orientation is correct, then perform other actions. 
            else:
            	#stop the north thruster as orientation is correct. 
            	stop_thrusters([NORTH])

            	'''
	            # Responsible for LONG
	            # Currently barebones
	            if tolerance(UUV.get_long_coord()):
	                if long_adjust_time == 0:
	                    long_adjust_time = time.monotonic()

	                change = adjust_speed([NORTH, SOUTH], long_adjust_time, long_scale, UUV.get_long_coord(), UUV.get_long_speed())

	                long_adjust_time    = change[0]
	                long_scale          = change[1]
	                long_adjustment     = change[2]

	                if not(long_sign_int == long_sign_current):
	                    long_sign_int = coord_sign(UUV.get_long_coord())
	                    long_scale -= 1

	            else:
	                stop_thrusters([NORTH, SOUTH])

	            # Responsible for LAT
	            # Currently barebones
	            if tolerance(UUV.get_lat_coord()):
	                if lat_adjust_time == 0:
	                    lat_adjust_time = time.monotonic()

	                change = adjust_speed([WEST, EAST], lat_adjust_time, lat_scale, UUV.get_lat_coord(), UUV.get_lat_speed())

	                lat_adjust_time     = change[0]
	                lat_scale           = change[1]
	                lat_adjustment      = change[2]

	                if not(lat_sign_int == lat_sign_current):
	                    lat_sign_int = coord_sign(UUV.get_lat_coord())
	                    lat_scale -= 1

	            else:
	                stop_thrusters([WEST, EAST])
				'''
        except KeyboardInterrupt:
            # Press CTRL+C to activate. Updates GPS position
            gps_update()
            print(LONG, LAT)
            print(wave_long_current, wave_lat_current)

if __name__ == '__main__':
    main()
