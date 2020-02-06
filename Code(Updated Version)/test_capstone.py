import Algorithm.Algorithm as cap
import math
import RPi.GPIO as IO
import time

#TESTCASES
TC = [(0,0), (0, 1), (0, -1), (1, 0), (-1, 0),
      (1, 1), (1, -1), (-1,1), (-1, -1)]

#checks the calculate_speeds function
assert cap.calculate_speeds(math.sqrt(3), 1)[0] == 0.00013333333333333334
assert cap.calculate_speeds(math.sqrt(3), 1)[1] == 6.66666666666667e-05

assert cap.calculate_speeds(1, math.sqrt(3))[1] == 0.00013333333333333334
assert cap.calculate_speeds(1, math.sqrt(3))[0] == 6.666666666666668e-05


######################
#TESTING CODE LOGIC WITH LEDS ON BREADBOARD
   
def turn_WE_low():
    cap.pwm_w.ChangeDutyCycle(0)
    cap.pwm_e.ChangeDutyCycle(0)

def turn_NS_low():
    cap.pwm_n.ChangeDutyCycle(0)
    cap.pwm_s.ChangeDutyCycle(0)
    
def turn_WE_backward():
    cap.pwm_w.ChangeFrequency(1)
    cap.pwm_w.ChangeDutyCycle(20)
    cap.pwm_e.ChangeFrequency(1)
    cap.pwm_e.ChangeDutyCycle(20)

def turn_WE_forward():
    cap.pwm_w.ChangeFrequency(10)
    cap.pwm_w.ChangeDutyCycle(20)
    cap.pwm_e.ChangeFrequency(10)
    cap.pwm_e.ChangeDutyCycle(20)

def turn_NS_backward():
    cap.pwm_n.ChangeFrequency(1)
    cap.pwm_n.ChangeDutyCycle(20)
    cap.pwm_s.ChangeFrequency(1)
    cap.pwm_s.ChangeDutyCycle(20)

def turn_NS_forward():
    cap.pwm_n.ChangeFrequency(10)
    cap.pwm_n.ChangeDutyCycle(20)
    cap.pwm_s.ChangeFrequency(10)
    cap.pwm_s.ChangeDutyCycle(20)
    
def change_pwm(pwm, freq, dc):
    pwm.ChangeFrequency(freq)
    pwm.ChangeDutyCycle(dc)
    
def main():
    
    #if the LED is flickering then we are moving forward
    #if the LED if beeping then we are moving backward
    for long_change, lat_change in TC:
        
        print("TEST CASE")
        print("LONG_CHANGE: ", long_change)
        print("LAT_CHANGE: ", lat_change)
        print("-"* 40)
        try:            
            #case1: NO CHANGE ---> Turn all thrusters off
            if(long_change == 0 and lat_change == 0):
                turn_NS_low()
                turn_WE_low()
            
            #case2: CHANGE IN LATITUDE
            elif(long_change == 0):
                #turn off North and South Thrusters
                turn_NS_low()
                #run West and East Thrusters
                if(lat_change >0):
                    turn_WE_backward()
                    
                elif(lat_change <0):
                    turn_WE_forward()
            
            #case3: CHANGE IN LONGTITUDE
            elif(lat_change == 0):
                #turn off WEST and EAST Thrusters
                turn_WE_low()
                #run NORTH and SOUTH Thrusters
                if(long_change >0):
                    
                    turn_NS_backward()
                    
                elif(long_change <0):
                    
                    turn_NS_forward()
                
            #case4: CHANGE IN BOTH 
            elif(long_change != 0 and lat_change != 0):
                
                if(long_change >0 and lat_change >0):
                    
                    turn_NS_backward()
                    
                    turn_WE_backward()
                                      
                elif(long_change >0 and lat_change <0):
                    
                    turn_NS_backward()
                    
                    turn_WE_forward()
                
                elif(long_change <0 and lat_change >0):
                    
                    turn_NS_forward()
                    
                    turn_WE_backward()
                    
                elif(long_change <0 and lat_change <0):
                    
                    turn_WE_forward()
                    
                    turn_NS_forward()
                        
        except KeyboardInterrupt:
            break
        
        time.sleep(15)
    IO.cleanup()

main()