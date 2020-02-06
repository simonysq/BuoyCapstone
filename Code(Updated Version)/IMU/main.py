import logging
import sys
import time
import RPi.GPIO as IO

from Adafruit_BNO055 import BNO055

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


#VARIBALES
CALIBRATION_PIN = 19
NORTH = 20
SOUTH = 26
EAST = 21
WEST = 16
PINS = [CALIBRATION_PIN, NORTH, SOUTH, EAST, WEST, 23]
POS_X = 0
POS_Y = 0

def calibrate_system():
    print("Please calibrate system!")
    IO.output(CALIBRATION_PIN, False)
    while True:
        sys, gyro, accel, mag = bno.get_calibration_status()
        print('Sys_cal=%d Gyro_cal=%d Accel_cal=%d Mag_cal=%d',
           sys, gyro, accel, mag)
        if(sys == 3 and gyro == 3 and accel == 3 and mag == 3):
            print("System calibrated")
            IO.output(CALIBRATION_PIN, True)
            break
    
        
def setup_GPIO():
    IO.setmode(IO.BCM)
    for pin in PINS:
        IO.setup(pin, IO.OUT)
    
def main():
    setup_GPIO()
    calibrate_system()
    position_data = open("position_data", "w+")
    position_data.write("a_x a_y POS_X POS_Y\n")
    POS_X = 0
    POS_Y = 0

    #give five seconds to get to initial position
    for x in range(5, 0, -1):
        print("%d seconds left.\n" %x)
        time.sleep(1)
        
    while True:
        try:
            a_x, a_y, a_z = bno.read_linear_acceleration()
            
            POS_X += a_x*0.0001
            POS_Y += a_y*0.0001
            
            position_data.write("%f " % a_x)
            position_data.write("%f " % a_y)
            position_data.write("%f " % POS_X)
            position_data.write("%f" % POS_Y)
            position_data.write("\n")
            
            print("POS_X: %f " % POS_X)
            print("POS_Y: %f" % POS_Y)
            print("\n")
            
            time.sleep(0.007)
            
        except KeyboardInterrupt:
            break
          
    print("Program ended")    
    position_data.close()
    IO.cleanup()    
    
main()
