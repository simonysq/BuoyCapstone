#This is the software-based simulation of the UUV's code.

import PySimpleGUI as sg
import math
import random
import time

###############################################
################## GUI ONLY ###################
# Change graph size
# zoom_scale changes Tab 2's graph by size/zoom_scale
size = 100
zoom_scale = 20

# Sets the initial states for the texts responsible for reporting thruster condition
default_a = "OFF"
default_b = "1500us"

# Draws the axis lines for key with scale
def graph_lines(key, scale, tick, tick_size, x_offset=0, y_offset=0):
    scaled = int(size / scale)

    key.DrawLine((-size, 0), (size, 0))
    key.DrawLine((0, -size), (0, size))

    for x in range(-scaled, scaled + 1, tick):
        key.DrawLine((x, -tick_size), (x, tick_size))
        if x != 0:
            key.DrawText(x + x_offset, (x, -tick), color="green")

    for y in range(-scaled, scaled + 1, tick):
        key.DrawLine((-tick_size, y), (tick_size, y))
        if y != 0:
            key.DrawText(y + y_offset, (-tick, y), color="blue")


def update_wave_currents():
    wave_long_current = random_change(33)
    wave_lat_current = random_change(33)


dispatch_dict = {"B1":update_wave_currents}


tab1_layout = [[sg.Graph(canvas_size=(400,400), graph_bottom_left=(-(size+5), -(size+5)),
                         graph_top_right=((size+5), (size+5)),
                    background_color="white", key="graph", tooltip="This is represents where the UUV is"
                                                                   " relative to its home position (0,0)")]]

tab2_layout = [[sg.Graph(canvas_size=(400,400), graph_bottom_left=(-(size/zoom_scale+1), -(size/zoom_scale+1)),
                         graph_top_right=((size/zoom_scale+1), (size/zoom_scale+1)),
                    background_color="white", key="zoom_graph", tooltip="This is represents where the UUV is"
                                                                   " relative to its home position (0,0)")]]
tab3_layout = [
    [sg.Text("Thruster North", pad=((5,30), (2,2)), justification="left"), sg.In("{} at {}".format(default_a, default_b),
        enable_events=False, disabled=True, key="M1", size=(20,2),pad=((3,25), (0,0)),
        tooltip="This shows the states of the thrusters")],

    [sg.Text("Thruster South", pad=((5,30), (0,0)), justification="left"), sg.In("{} at {}".format(default_a, default_b),
        enable_events=False, disabled=True, key="M2", size=(20,2), pad=((0,25), (2,2)),
        tooltip="This shows the states of the thrusters")],

    [sg.Text("Thruster West", pad=((5,30), (0,0)), justification="left"), sg.In("{} at {}".format(default_a, default_b),
        enable_events=False, disabled=True, key="M3", size=(20,2), pad=((3,25), (2,2)),
        tooltip="This shows the states of the thrusters")],

    [sg.Text("Thruster East", pad=((5,30), (0,0)), justification="left"), sg.In("{} at {}".format(default_a, default_b),
        enable_events=False, disabled=True, key="M4", size=(20,2), pad=((7,25), (2,2)),
        tooltip="This shows the states of the thrusters")],

    [sg.Text("Orientation", pad=((5, 30), (0, 0)), justification="left"), sg.In("{}°".format("0"),
        enable_events=False, disabled=True, key="M6", size=(20, 2), pad=((21, 25), (2, 2)),
        tooltip="This shows the orientation of the UUV")],

    [sg.Text("_"*80, pad=((0,0),(0,0)))],

    [sg.Text("UUV Speed - Latitude", pad=((5, 30), (0, 0)), justification="left"),
     sg.In("{}".format(0),
           enable_events=False, disabled=True, key="M9", size=(20, 2), pad=((39, 25), (2, 2)),
           tooltip="Latitude Speed")],

    [sg.Text("UUV Speed - Longtitude", pad=((5, 30), (0, 0)), justification="left"),
     sg.In("{}".format(0),
           enable_events=False, disabled=True, key="M10", size=(20, 2), pad=((25, 25), (2, 2)),
           tooltip="Longitude Speed")],

    [sg.Text("UUV Coordinate - Latitude", pad=((5, 30), (0, 0)), justification="left"),
     sg.In("{}".format(0),
           enable_events=False, disabled=True, key="M11", size=(20, 2), pad=((14, 25), (2, 2)),
           tooltip="Latitude")],

    [sg.Text("UUV Coordinate - Longtitude", pad=((5, 30), (0, 0)), justification="left"),
     sg.In("{}".format(0),
           enable_events=False, disabled=True, key="M12", size=(20, 2), pad=((0, 25), (2, 2)),
           tooltip="Longitude")]
               ]

tab4_layout = [
    [sg.Text("Wave Current - Latitude", pad=((5, 30), (2, 2)), justification="left"),
     sg.In("{0:.4f} m/s".format(0),
           enable_events=False, disabled=True, key="M7", size=(20, 2), pad=((10, 25), (0, 0)),
           tooltip="Wave Current - Latitude")],

    [sg.Text("Wave Current - Longitude", pad=((5, 30), (0, 0)), justification="left"),
     sg.In("{0:.4f} m/s".format(0),
           enable_events=False, disabled=True, key="M8", size=(20, 2), pad=((0, 25), (2, 2)),
           tooltip="Wave Current - Longitude")],

    [sg.Button("Generate New Wave Currents", tooltip="Sets new wave currents", key="B1")],

    [sg.Text("_" * 80, pad=((0, 0), (0, 0)))],

    [sg.Text("Home Latitude", pad=((5, 30), (2, 2)), justification="left"),
     sg.In("{0:.4f}°".format(0),
           enable_events=False, disabled=True, key="M13", size=(20, 2), pad=((10, 25), (0, 0)),
           tooltip="Default is [0,0]")],

    [sg.Text("Home Longitude", pad=((5, 30), (0, 0)), justification="left"),
     sg.In("{0:.4f}°".format(0),
           enable_events=False, disabled=True, key="M14", size=(20, 2), pad=((0, 25), (2, 2)),
           tooltip="Default is [0,0]")],

    [sg.Button("Generate New Home Coordinates", tooltip="Sets new coordinates", key="B2")],

    [sg.Input(tooltip="Format is x, y", key="In1"), sg.Button("Submit", key="B3")]
]

layout = [[sg.TabGroup([[sg.Tab("Tab 1", tab1_layout, tooltip="Graph"), sg.Tab("Tab 2", tab2_layout, tooltip="Graph"),
                         sg.Tab("Tab 3", tab3_layout), sg.Tab("Tab 4", tab4_layout, tooltip="Simulated Variables")]],
                       )], [sg.Text("_"*80, pad=((0,0),(0,0)))],[sg.MLine("Initialized...\n", key="M5", size=(60,4))]]

window = sg.Window("UUV - Version 0.9.9.6", layout, grab_anywhere=True).Finalize()

# Assigning names to GUI elements
graph = window["graph"]
zoom_graph = window["zoom_graph"]

# Calls graph_lines to draw axis
graph_lines(graph, 1, 10, 3)
graph_lines(zoom_graph, zoom_scale, 2, .5)

# Used to control UUV location on graph
point_id = False
point_id2 = False

# Flag that tells the GUI to initialize a specific code once at startup
is_initialized = False

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

    # For testing only
    def force_orientation(self, orientation):
        self.orientation = orientation

    def get_orientation(self):
        return self.orientation


# Sets up a class responsible for controlling each individual thruster's speed
class Thruster:
    def __init__(self, name, speed, state):
        self.name = name
        self.speed = speed
        self.state = state

    def get_speed(self):
        return self.speed

    def set_speed(self, speed):
        self.speed = speed

    def get_name(self):
        return self.name

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = state


##############################################
################# Variables ##################
# Defines the maximum speed that the thrusters can go at and what the STOP signal is
# Actual hardware has max forward pulsewidth of 1900us and reverse of 1100us and a stop of 1500us
# In simulation, we map 1900us to 4 and 1100us to -4.
MAX_FORWARD_SPEED = 4
NEUTRAL = 0
MAX_BACKWARD_SPEED = -4

# Splits up the 4 thrusters' adjustment speeds into 2 sets: LONG (NS) and LAT (WE)
# ADJUSTMENT is added to NEUTRAL, which is what causes the thrusters to spin either forward or backwards
long_adjustment = 0
lat_adjustment = 0

# Stores the UUV's axis' signs. NORTH/EAST is positive. WEST/SOUTH is negative.
# These values only changes once sign_int's value is different than sign_current
# These values are set prior to any speed adjustment
long_sign_int = 0
lat_sign_int = 0

# These values are set as the system is running and is compared against sign_int
long_sign_current = 0
lat_sign_current = 0

# Initializes adjustment_time, which is used to delay changes in thruster speeds
long_adjust_time = 0
lat_adjust_time = 0

# Initializes time_since_adjustment, which is used to track how long as direction correction as been going on
long_time_since_adjustment  = 0
lat_time_since_adjustment   = 0

# Defaults scale to 1
long_scale = 1
lat_scale = 1

# Previous Location
long_old = 0
lat_old = 0

# Flag for if the UUV is currently orientating or not
is_orientating = False

##############################################
################## Objects ###################

# Sets up the four thrusters into instances: thrusters are all off.
NORTH = Thruster("NORTH", NEUTRAL, "OFF")
SOUTH = Thruster("SOUTH", NEUTRAL, "OFF")
EAST = Thruster("EAST", NEUTRAL, "OFF")
WEST = Thruster("WEST", NEUTRAL, "OFF")

##############################################
################## Arrays ####################
PINS = [NORTH, SOUTH, WEST, EAST]


##############################################
##### Simulation Functions and Variables #####

# Functions
def random_change(step_value):
    # generates and returns a value between |100/step_value| with a step of 1/step_value
    return random.randrange(-100, 100, 1) / step_value


# Variables
# Sets up the home location to 0, 0 -- corresponds to a XY coordinate system
HOME_LAT = 0
HOME_LONG = 0

# Calls random_change for each one, generating a random LAT/LONG to start off
LAT = random_change(5)
LONG = random_change(5)

# Wave Current (from -3.33 to +3.33 m/s)
wave_lat_current = random_change(33)
wave_long_current = random_change(33)

# Orientation: attach sensor at front of UUV (eg. north thruster), select random angle from 0 to 360 degrees
# to be oriented front of UUV must face north, (90 degrees out of 360 degrees)
ORIENT = random.randrange(360)

# UUV speed
UUV_long_speed = 0
UUV_lat_speed = 0

# Initializes the Buoy object
# ORIENT - ORIENT else it'll append to orientation twice (second time is when orientation_update is called)
UUV = Buoy(LAT, LONG, wave_lat_current, wave_long_current, ORIENT - ORIENT)

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
    return window["M5"].update("Fix detected, continuing...\n", append=True)


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
        PIN.set_state("OFF")
        if (PIN == NORTH) or (PIN == SOUTH):
            long_scale = 1
        if (PIN == EAST) or (PIN == WEST):
            lat_scale = 1
    time.sleep(0.01)


# Sets the relevant thrusters to a speed defined by DUTYCYCLE
def run_thrusters(PINS, DUTYCYCLE):
    for PIN in PINS:
        PIN.set_speed(DUTYCYCLE)
        PIN.set_state("ON")
    time.sleep(0.1)


# Tolerance compares the magnitudes of the LONG/LAT values with a fixed value,
# which corresponds to the +- 10ft specification
def tolerance(coord, home_coord):
    tolerance = 1.5
    if (home_coord - tolerance) < coord < (home_coord + tolerance):
        return False
    else:
        return True


# Can be optimized
# Takes in thrusters that need to be adjusted
# t = time of last adjustment
# scale = how much should the adjustment to speed be
# coord defines if the change is either a positive or negative
def adjust_speed(PINS, delay, scale, coord, speed, time_elapsed, home_coord):
    # Refreshes the UI
    window.Refresh()

    adjustment = scale

    if (time.monotonic() - delay > 1) or scale == 1:
        if time_elapsed >= 30:
            scale *= 2
        elif time_elapsed >= 10:
            scale *= 1.5
        else:
            scale += 1

        if scale >= 100:
            scale = 100

        adjustment = 4 * math.exp(scale / 10 - 5) / (math.exp(scale / 10 - 5) + 1)

        delay = time.monotonic()

        if coord > home_coord:
            run_thrusters(PINS, NEUTRAL - adjustment)
        else:
            run_thrusters(PINS, NEUTRAL + adjustment)

        return [delay, scale, adjustment]

    else:
        return [delay, scale, adjustment]


#for thruster to orient the UUV, active the north thruster to cause the UUV to rotate.
#To be in correct orientation, orientation = 90 degrees: which means that vehicle is facing north.
#If in quadrant I or IV, north thruster will spin forward and rotate counterclockwise. Opposite applies to II and III
#Three speed levels: more than 90 degrees away(speed = 2), more than 15 degrees away (speed = 0.75), less than 15 (speed = 0.23)
def rotateForwOrBack(PIN, angle):
    # Refreshes the UI
    window.Refresh()

    # quadrant I
    if angle < 90:
        if angle >= 75:
            run_thrusters(PIN, 0.23)
        else:
            run_thrusters(PIN, 0.75)
        # quadrant IV
    elif angle > 270:
        run_thrusters(PIN, 2)
        # quadrant II
    elif angle > 90 and angle <= 180:
        if (angle - 90) <= 15:
            run_thrusters(PIN, -0.23)
        else:
            run_thrusters(PIN, -0.75)
        # quadrant III
    elif angle > 180 and angle <= 270:
        run_thrusters(PIN, -2)


# Checks the sign of the current coordinate
def coord_sign(coord):
    if coord >= 0:
        return 1
    else:
        return -1


# Sets up simulator parameters at startup
def initialize():
    global lat_sign_int, long_sign_int
    # Calls for GPS location
    window.Refresh()

    window["M5"].update("Waiting for a fix...\n", append=True)
    window["M7"].update("{0:.4f} m/s".format(wave_lat_current))
    window["M8"].update("{0:.4f} m/s".format(wave_long_current))

    gps_fix()

    # Updates current position
    gps_update()

    # Update Orientation.
    orientation_update()

    # Compares change from HOME location [0, 0] (default)
    lat_sign_int = coord_sign(LAT + HOME_LAT)
    long_sign_int = coord_sign(LONG + HOME_LONG)

    # update the UUV values, notice that UUV is moving at speed of currents.
    UUV.set_lat_coord(LAT)
    UUV.set_long_coord(LONG)
    UUV.set_lat_speed(wave_lat_current)
    UUV.set_long_speed(wave_long_current)
    UUV.set_orientation(ORIENT)

    #for test purposes
    while 1:
        test_orient = random.randrange(86, 94, 1)
        if test_orient > 93 or test_orient < 87:
            UUV.force_orientation(test_orient)
            break


# Calls to update Tab 3 Data
def update_tab3():
    window.Refresh()

    window["M1"].update("{0} at {1:.2f}μs".format(NORTH.get_state(), NORTH.get_speed() * 100 + 1500))
    window["M2"].update("{0} at {1:.2f}μs".format(SOUTH.get_state(), SOUTH.get_speed() * 100 + 1500))
    window["M3"].update("{0} at {1:.2f}μs".format(WEST.get_state(), WEST.get_speed() * 100 + 1500))
    window["M4"].update("{0} at {1:.2f}μs".format(EAST.get_state(), EAST.get_speed() * 100 + 1500))
    window["M6"].update("{0:.4f}°".format(UUV.get_orientation()))
    window["M9"].update("{0:.2f} m/s".format(UUV.get_lat_speed()))
    window["M10"].update("{0:.2f} m/s".format(UUV.get_long_speed()))
    window["M11"].update("{0:.4f}°".format(UUV.get_lat_coord()))
    window["M12"].update("{0:.4f}°".format(UUV.get_long_coord()))


def main():
    global adjustment_time, lat_adjustment, long_adjustment, lat_adjust_time, long_adjust_time
    global lat_scale, long_scale, orientation_scale, is_orientating, long_sign_int, lat_sign_int
    global long_time_since_adjustment, lat_time_since_adjustment
    global point_id, point_id2, default_a, default_b, is_initialized, wave_lat_current, wave_long_current
    global HOME_LAT, HOME_LONG

    lat_fine_count      = 0
    long_fine_count     = 0

    # Main Loop
    while (1):

        # GUI STARTS HERE
        event, values = window.Read(timeout=10)

        if event in (None, "Exit"):
            break

        if event == "B1":
            wave_lat_current    = random_change(33)
            wave_long_current   = random_change(33)

            window["M7"].update("{0:.4f} m/s".format(wave_lat_current))
            window["M8"].update("{0:.4f} m/s".format(wave_long_current))

        if event == "B2":
            HOME_LAT            = random_change(5)
            HOME_LONG           = random_change(5)

            long_scale          = 1
            lat_scale           = 1

            long_time_since_adjustment  = 0
            lat_time_since_adjustment   = 0

            window["M13"].update("{0:.4f}°".format(HOME_LAT))
            window["M14"].update("{0:.4f}°".format(HOME_LONG))

            zoom_graph.Erase()
            graph_lines(zoom_graph, zoom_scale, 2, .5, int(HOME_LAT), int(HOME_LONG))

        if event == "B3":
            new_coords = window["In1"].Get().split(",")
            window["In1"].Update("")

            try:
                if len(new_coords) != 2:
                    window["M5"].Update("Invalid entry. Format is x,y. No spaces.", text_color="red")
                else:
                    HOME_LAT            = float(new_coords[0])
                    HOME_LONG           = float(new_coords[1])
            except ValueError:
                window["M5"].Update("Invalid entry. Format is x,y. No spaces.", text_color="red")

            window.Read(timeout=2000)

            long_scale          = 1
            lat_scale           = 1

            long_time_since_adjustment  = 0
            lat_time_since_adjustment   = 0

            window["M13"].update("{0:.4f}°".format(HOME_LAT))
            window["M14"].update("{0:.4f}°".format(HOME_LONG))

            zoom_graph.Erase()
            graph_lines(zoom_graph, zoom_scale, 2, .5, int(HOME_LAT), int(HOME_LONG))

        if point_id != False:
            graph.DeleteFigure(point_id)

        if point_id2 != False:
            zoom_graph.DeleteFigure(point_id2)

        # Updates Tab 3 Data
        update_tab3()

        # Draws the UUV's location on the graph(s)
        point_id = graph.DrawPoint((UUV.get_lat_coord(), UUV.get_long_coord()), size=5, color="black")
        point_id2 = zoom_graph.DrawPoint((UUV.get_lat_coord() - HOME_LAT,
                                          UUV.get_long_coord() - HOME_LONG), size=.5, color="black")

        zoom_graph.DrawCircle((0,0), 3)

        # If first iteration, initialize
        if not is_initialized:
            is_initialized = True
            initialize()

            window.Read(timeout=2000)
        # GUI ENDS HERE

        # Tracks how long as it been
        last_runtime = time.monotonic()

        try:
            # Refreshes the UI
            window.Refresh()

            current_time = time.monotonic()
            # Compare coordinates
            long_sign_current = coord_sign(UUV.get_long_coord() + HOME_LONG)
            lat_sign_current = coord_sign(UUV.get_lat_coord() + HOME_LAT)

            # Delay update for a maximum of 0.01s
            if current_time - last_runtime > 0.01:
                time.sleep(0.01)

            else:
                time.sleep(0.01 - (current_time - last_runtime))
            wait = time.monotonic() - last_runtime

            # set current speed and location of UUV
            UUV.set_long_speed(NORTH.get_speed() + wave_long_current)
            UUV.set_lat_speed(WEST.get_speed() + wave_lat_current)
            UUV.set_long_coord(UUV.get_long_speed() * wait)
            UUV.set_lat_coord(UUV.get_lat_speed() * wait)

            # Refreshes the GUI for user input, wait time of 20ms
            window.Refresh()

            # set new orientation of UUV:
            # w = v / r: v is in m/s, r = 5 cm so 0.05m
            if is_orientating == True:
                omega = NORTH.get_speed() / 0.05
                # angle change = omega * change in time:
                changeAngle = omega * wait
                UUV.set_orientation(changeAngle)
                window["M5"].update("Currently orientating...\nCurrent orientation at: {0:.2f}°"
                                    .format(UUV.get_orientation()), text_color="black")

            '''print("{0:14s}{1:>5f}\n{2:14s}{3:>5f}\n{4:14s}{5:>5f}\n".format("Lat Post:", UUV.get_lat_coord(),
                                                                            "Long Post:", UUV.get_long_coord(),
                                                                            "Orientation:", UUV.get_orientation()))'''

            last_runtime = current_time

            # Before moving, must ensure orientation of UUV is correct: approximate 90 degrees (+- 3 degrees)
            tempOrientation = UUV.get_orientation()
            if tempOrientation > 93 or tempOrientation < 87:
                is_orientating = True
                rotateForwOrBack([NORTH], tempOrientation)

            # only if orientation is correct, then perform other actions.
            else:

                # Refreshes the GUI for user input, wait time of 20ms
                window.Refresh()

                # stop the north thruster as orientation is correct.
                is_orientating = False
                if NORTH.get_speed() in (-0.23, 0.23):
                    stop_thrusters([NORTH])

                # Outputs where the UUV is exactly
                window["M5"].update("Current direction is...\nLatitude: {0:.4f}°\nLongitude: {1:.4f}°"
                                    .format(UUV.get_lat_coord(), UUV.get_long_coord()), text_color="black")

                # Responsible for LONG
                # Currently barebones
                if tolerance(UUV.get_long_coord(), HOME_LONG):
                    if long_adjust_time == 0:
                        long_adjust_time = time.monotonic()

                    if long_time_since_adjustment == 0:
                        long_time_since_adjustment = time.monotonic()

                    adjustment_time = time.monotonic() - long_time_since_adjustment

                    change = adjust_speed([NORTH, SOUTH], long_adjust_time, long_scale, UUV.get_long_coord(),
                                          UUV.get_long_speed(), adjustment_time, HOME_LONG)
                    long_adjust_time = change[0]
                    long_scale = change[1]
                    long_adjustment = change[2]
                    if not (long_sign_int == long_sign_current):
                        long_sign_int = coord_sign(UUV.get_long_coord())

                else:
                    stop_thrusters([NORTH, SOUTH])
                    if long_scale < 0:
                        long_scale = 0
                    else:
                        long_fine_count += 1
                        if long_fine_count >= 5:
                            long_scale -= 2
                            long_fine_count = 0
                        else:
                            long_scale -= .75

                    long_time_since_adjustment = time.monotonic()

                # Refreshes the GUI for user input, wait time of 20ms
                window.Refresh()

                # Responsible for LAT
                # Currently barebones
                if tolerance(UUV.get_lat_coord(), HOME_LAT):
                    if lat_adjust_time == 0:
                        lat_adjust_time = time.monotonic()

                    if lat_time_since_adjustment == 0:
                        lat_time_since_adjustment = time.monotonic()

                    adjustment_time = time.monotonic() - lat_time_since_adjustment

                    change = adjust_speed([WEST, EAST], lat_adjust_time, lat_scale, UUV.get_lat_coord(),
                                          UUV.get_lat_speed(), adjustment_time, HOME_LAT)
                    lat_adjust_time = change[0]
                    lat_scale = change[1]
                    lat_adjustment = change[2]
                    if not (lat_sign_int == lat_sign_current):
                        lat_sign_int = coord_sign(UUV.get_lat_coord())

                else:
                    stop_thrusters([WEST, EAST])
                    if lat_scale < 0:
                        lat_scale = 0
                    else:
                        lat_fine_count += 1
                        if lat_fine_count >= 10:
                            lat_scale -= 2
                            lat_fine_count = 0
                        else:
                            lat_scale -= .75

                    lat_time_since_adjustment = time.monotonic()

                # Refreshes the GUI for user input, wait time of 20ms
                window.Refresh()

        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    main()
