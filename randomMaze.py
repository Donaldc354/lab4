import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import decimal
import math
import Adafruit_PCA9685
import csv
import random

import cv2 as cv
#from ThreadedWebcam import ThreadedWebcam
#from UnthreadedWebcam import UnthreadedWebcam

#Global Variables
startTime = time.time()
currentTime = 0

fps = 0.0
prev = 0.0
x_pos = 0
pi = 3.142

kpValue = 0.9

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second.
LSERVO = 0
RSERVO = 1

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b

# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

# Shutdown all sensors
GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)

time.sleep(0.01)

# Initialize all sensors
lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)

# Connect the left sensor and start measurement
GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the right sensor and start measurement
GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

def ctrlC(signum, frame):
    print("Exiting")
    
    GPIO.cleanup()
    # Write an initial value of 1.5, which keeps the servos stopped.
    # Due to how servos work, and the design of the Adafruit library,
    # The value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

    exit()

def readCSV():
    global LWSpeed, RWSpeed
    with open('LeftSpeedCalibration.csv', newline='') as LeftCalibrate:
        lReader = csv.reader(LeftCalibrate)
        LWSpeed = dict(map(float,x) for x in lReader) # pulls in each row as a key-value pair
        
    with open('RightSpeedCalibration.csv', newline= '') as RightCalibrate:
        rReader = csv.reader(RightCalibrate)
        RWSpeed = dict(map(float,x) for x in rReader) # pulls in each row as a key-value pair

def setDifference(speed):
    diff = speed - 1.5
    return 1.5 - diff
        
# Sets speed of motors in Inches per econd
def setSpeedsIPS(ipsLeft, ipsRight):
      # Converting inches per second into revolutions per second
      rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
      rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

      if rpsLeft < 0:
            rpsLeft = 0 - rpsLeft
      if rpsRight < 0:
            rpsRight = 0 - rpsRight

      # Calculating pwm values from the respective dictionaries
      lPwmValue = float(LWSpeed[rpsLeft])
      rPwmValue = float(RWSpeed[rpsRight])

      if ipsLeft < 0 or ipsRight < 0:
            # Setting appropiate speeds to the servos when going forwards
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue)    / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))
      elif ipsLeft >= 0 or ipsRight >= 0:
            # Setting apporpiate speeds to the servos when going backwards
            pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))
def spinIPS(ipsLeft, ipsRight):
      # Converting inches per second into revolutions per second
      rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
      rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

      if rpsLeft < 0:
            rpsLeft = 0 - rpsLeft
      if rpsRight < 0:
            rpsRight = 0 - rpsRight

      # Calculating pwm values from the respective dictionaries
      lPwmValue = float(LWSpeed[rpsLeft])
      rPwmValue = float(RWSpeed[rpsRight])

      if ipsLeft < 0 or ipsRight < 0:
            # Setting appropiate speeds to the servos when going forwards
            pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))
      elif ipsLeft >= 0 or ipsRight >= 0:
            # Setting apporpiate speeds to the servos when going backwards
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))

# Sets boundary speed for robot movement
def saturationFunction(ips):
	controlSignal = ips
	if controlSignal > 4.0:
		controlSignal = 4.0
	elif controlSignal < -4.0:
		controlSignal = -4.0
	return controlSignal

# Sets boundary speed for robot movement
def saturationFunctionGoalFacing(ips):
	controlSignal = ips
	if controlSignal > 1.0:
		controlSignal = 1.0
	elif controlSignal < -1.0:
		controlSignal = -1.0
	return controlSignal

###### OPEN CV FUNCTIONS #######

def goalSearch():
    if len(keypoints) >= 1:
    	error = 280 - x_pos

    	controlSignal = kpValue * error

    	newSignal = saturationFunctionGoalFacing(controlSignal)

    	spinIPS(newSignal, newSignal)

    else:
        pwm.set_pwm(LSERVO, 0, math.floor(1.515 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.515 / 20 * 4096))


    print("IM GOING THE GOALLLLLL!!!!")
    #sensorCount = 0

	# Gets Distance From Sensor
    fDistance = fSensor.get_distance()
	# Converts readings from milimeters to inches
    inchDistance = fDistance * 0.03937
   	# 0.394 is the conversion rate from cm to inches Determining error amount

   	# fError is the calculated respective error value aka the e(t) value
    error = 5.0 - inchDistance

    # Control Signal aka u(t)  = Kp * e(t)
    controlSignal = kpValue * error

    # Calculating new control signal value by running control signal through saturation function
    newSignal = saturationFunction(controlSignal)

    setSpeedsIPS(newSignal, newSignal)

def rightTurn():
    # Gets Distance From Sensor
    fDistance = fSensor.get_distance()
    lDistance = lSensor.get_distance()
	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    linchDistance = lDistance * 0.03937
   	# 0.0394 is the conversion rate from mm to inches Determining error amount

    # Loop turns Robot as long as there is an object in front of it
    while finchDistance < 8 and linchDistance < 8:
        pwm.set_pwm(LSERVO, 0, math.floor(1.52 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.52 / 20 * 4096))
	    # Gets Distance From Sensor
        fDistance = fSensor.get_distance()
        lDistance = lSensor.get_distance()
	    # Converts readings from milimeters to inches
        inchDistance = fDistance * 0.03937
        linchDistance = lDistance * 0.03937
   	    # 0.0394 is the conversion rate from mm to inches Determining error amount

    pwm.set_pwm(LSERVO, 0, math.floor(1.50 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.50 / 20 * 4096))
#End Right Turn Not tested yet!!!!!!!!

def leftTurn():
    # Gets Distance From Sensor
    fDistance = fSensor.get_distance()
	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    rinchDistance = rDistance * 0.03937
   	# 0.0394 is the conversion rate from mm to inches Determining error amount

    # Loop turns Robot as long as there is an object in front of it
    while finchDistance < 8 and rinchDistance < 8:
        pwm.set_pwm(LSERVO, 0, math.floor(1.48 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.48 / 20 * 4096))
	    # Gets Distance From Sensor
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
	    # Converts readings from milimeters to inches
        finchDistance = fDistance * 0.03937
        rinchDistance = rDistance * 0.03937
   	    # 0.0394 is the conversion rate from mm to inches Determining error amount

    pwm.set_pwm(LSERVO, 0, math.floor(1.50 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.50 / 20 * 4096))
#End Left Turn not Tested yet!!!!!!!

def reverse():
    # Gets Distance From Sensor
    fDistance = fSensor.get_distance()
    lDistance = lSensor.get_distance()
    rDistance = rSensor.get_distance()
	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    linchDistance = lDistance * 0.03937
    rinchDistance = rDistance * 0.03937
   	# 0.0394 is the conversion rate from mm to inches Determining error amount

    # Loop turns Robot as long as there is an object in front of it
    while finchDistance < 8 and rinchDistance < 8 and linchDistance < 8:
        pwm.set_pwm(LSERVO, 0, math.floor(1.48 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.48 / 20 * 4096))
	    # Gets Distance From Sensor
        fDistance = fSensor.get_distance()
	    # Converts readings from milimeters to inches
        inchDistance = fDistance * 0.03937
   	    # 0.0394 is the conversion rate from mm to inches Determining error amount

    pwm.set_pwm(LSERVO, 0, math.floor(1.50 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.50 / 20 * 4096))
#End Reverse Turn not ready yet!!!!!!!

def chance():
    if random.radint(0,100) < 50:
        return rightTurn()
    return leftTurn()

def motionToGoal():
    print("IM GOING THE GOALLLLLL!!!!")

	# Gets Distance From Sensor
    fDistance = fSensor.get_distance()
    lDistance = lSensor.get_distance()
    rDistance = rSensor.get_distance()

	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    linchDistance = lDistance * 0.03937
    rinchDistance = rDistance * 0.03937
   	# 0.394 is the conversion rate from cm to inches Determining error amount

    setSpeedsIPS(2,2)

    if finchDistance < 6:
        if linchDistance < 6:
            print("Turning Right")
            setSpeedsIPS(2,1)
            #rightTurn()
            time.sleep(0.25)
            #wallFollowing()
    
    setSpeedsIPS(2,2)

    if finchDistance < 6:
        if rinchDistance < 6:
            print("Turning Left")
            #leftTurn()
            setSpeedsIPS(setSpeeds)
            time.sleep(0.25)
            #wallFollowing()

    setSpeedsIPS(2,2)

    if finchDistance < 6:
        if rinchDistance > 8:
            if linchDistance > 8:
                print("Turning Left!")
                setSpeedsIPS(0,2)
                #leftTurn()
                time.sleep(5)
                #wallFollowing()
    
    setSpeedsIPS(2,2)

def center():
    
    #measure distances..
    fDistance = fSensor.get_distance()
    lDistance = lSensor.get_distance()
    rDistance = rSensor.get_distance()

	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    linchDistance = lDistance * 0.03937
    rinchDistance = rDistance * 0.03937

    pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.45 / 20 * 4096))
    while finchDistance > 5:
        print("front Sensor greater than 6")
        fDistance = fSensor.get_distance()
        lDistance = lSensor.get_distance()
        rDistance = rSensor.get_distance()
        print("Front Sensor: ", fDistance)

        # Converts readings from milimeters to inches
        finchDistance = fDistance * 0.03937
        linchDistance = lDistance * 0.03937
        rinchDistance = rDistance * 0.03937

        if linchDistance < 5:
            print("left Sensor, " )
            pwm.set_pwm(LSERVO, 0, math.floor(1.58 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.45 / 20 * 4096))

        if rinchDistance < 5:
            pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.43 / 20 * 4096))

        if linchDistance > 12 and rinchDistance > 12:
            print("front Sensor greater than 6")
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
            break

        if linchDistance > 12:
            print("front Sensor greater than 12")
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
            break
        
        if rinchDistance > 12:
            print("front Sensor greater than 12")
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
            break
    
    if finchDistance < 3:
        print("front Sensor less than 4")
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    
    
    #setSpeedsIPS(3,3)
    #time.sleep(3)
        
    
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

    if linchDistance > 13:
        pwm.set_pwm(LSERVO, 0, math.floor(1.47 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.47 / 20 * 4096))
        time.sleep(2)

    if finchDistance < 5.0:
        sensorCount += 1
        if sensorCount > 4:
            frontDist()
    else :
        sensorCount = 0




# Attach the Ctrl+C signal interrupt and initialize encoders
signal.signal(signal.SIGINT, ctrlC)

# Initialized servos to zero movement
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
time.sleep(2)

# Imports dictionary from the calibration CSV file
readCSV()


    ########################## MAIN LINE CODE ####################################

pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

startFlag = False
selectCommand = ' '
#pwm.set_pwm(LSERVO, 0, math.floor(1.45 / 20 * 4096))
#pwm.set_pwm(RSERVO, 0, math.floor(1.45 / 20 * 4096))
#time.sleep(0.9)
# Holds program until command value is entered
while selectCommand != 's':
      selectCommand = input("Please enter \'s\' to begin robot movement: ")

startFlag = True


# 
while startFlag:
    # Calculate FPS

    #pwm.set_pwm(LSERVO, 0, math.floor(1.48 / 20 * 4096))
    #pwm.set_pwm(RSERVO, 0, math.floor(1.48 / 20 * 4096))

    #measure distances..
    fDistance = fSensor.get_distance()
    lDistance = lSensor.get_distance()
    rDistance = rSensor.get_distance()

	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    linchDistance = lDistance * 0.03937
    rinchDistance = rDistance * 0.03937



    #Constant following

    #if rDistance < 3:
       # setSpeedsIPS(1,2)

    #if lDistance < 3:
       # setSpeedsIPS(2,1)


   # while fDistance < 3:
        #while lDistance < 3:
            #setSpeedsIPS(2,1)
            #sleep(pi/2)
       # setSpeedsIPS(1,2)

    #while fDistance < 3:
        #while rDistance < 3:
           # setSpeedsIPS(1,2)
            #sleep(pi/2)
        #setSpeedsIPS(2,1)
    center()


    startFlag = False
    #pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    #pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    now = time.time()
    

# Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()