import os
import sys
import time
from abc import ABC, abstractmethod
from time import sleep

import RPi.GPIO as GPIO
import numpy as np
import pigpio
import smbus as smbus
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pig = pigpio.pi()

pi = np.pi
c = np.cos
s = np.sin

delay = 0.000005
kp = 250
iterationCount = 30


class Joint:
    @abstractmethod
    def turn(self, angle):
        pass


class ServoJoint(Joint):

    def __init__(self, controlPin, rangeOfMotion=180):
        self.controlPin = controlPin
        self.rangeOfMotion = rangeOfMotion

    def turn(self, angle):
        frequency = self.angleToFrequency(angle)
        pig.set_servo_pulsewidth(self.controlPin, frequency)

    def angleToFrequency(self, angle):
        if self.rangeOfMotion == 270:
            return (1000 / 135) * angle + 1500
        else:
            return (1000 / 90) * angle + 1500
    
    def direction(self, value):
        if value < 1500:
            return 1
        else:
            return -1
    
    def controlledTurn(self, angle):
        
        frequencyCurrent = pig.get_servo_pulsewidth(self.controlPin)
        frequencyTarget = self.angleToFrequency(angle)
        print(frequencyTarget)
        
        while abs(frequencyCurrent - frequencyTarget) > 3:
            delta = abs(frequencyCurrent - frequencyTarget)
            print(round(delta, 2), end="    ")
            frequency = delta/3
            print(round(frequency, 2), end="    ")
            frequency = frequencyCurrent + frequency*self.direction(delta)
            print(round(frequency, 2))
            pig.set_servo_pulsewidth(self.controlPin, frequency)
            frequencyCurrent = pig.get_servo_pulsewidth(self.controlPin)
            sleep(0)
            
    def pid(self, counter, target):
        result = ((delay) / ((target - counter) / target))
        if result < delay:
            result = delay
        elif result > delay*100:
            result = delay*100
        return result
    
    def manualTurn(self, angle):

        if angle < 1500:
            direct = 1
        else:
            direct = -1

        delay = 1
        
        frequencyCurrent = pig.get_servo_pulsewidth(self.controlPin)
        frequencyTarget = self.angleToFrequency(angle)
        print(frequencyTarget)
        delta_tot = abs(frequencyCurrent - frequencyTarget)
        

        for x in range(int(delta_tot)):
            delta = abs(frequencyCurrent - frequencyTarget)
            delay = self.pid(x, delta_tot)
            frequency = frequencyCurrent + delta*self.direction(delta)
            print("frequency: ", round(frequency, 2))
            pig.set_servo_pulsewidth(self.controlPin, frequency)
            sleep(delay)
            frequencyCurrent = pig.get_servo_pulsewidth(self.controlPin)
            #print(delay, "    ", x)
        print("done")
            
            
        


class StepperJoint(Joint):
    stepPin = 0
    directionPin = 0
    SoA = 0

    def __init__(self, stepPin, directionPin):
        self.stepPin = stepPin
        self.directionPin = directionPin
        GPIO.setup(stepPin, GPIO.OUT)
        GPIO.setup(directionPin, GPIO.OUT)

    def directionCalculation(self, value):

        if value > 0:
            GPIO.output(self.directionPin, GPIO.HIGH)
        else:
            GPIO.output(self.directionPin, GPIO.LOW)

    def stepToAngle(self, steps):
        return (33333 / steps) * 90

    def angleToStep(self, angle):
        return (33333 / 90) * angle

    def pid(self, counter, target):
        result = ((delay) / ((target - counter) / target))
        if result < delay:
            result = delay
        elif result > delay*100:
            result = delay*100
        return result

    def manualTurn(self, angle):

        if angle > 0:
            direct = 1
        else:
            direct = -1

        self.SoA = self.SoA + angle
        print("state of art: ", self.SoA)

        self.directionCalculation(direct)

        angle = abs(angle)
        steps = self.angleToStep(angle)
        print("steps: ", steps)

        for x in range(int(steps)):
            delay = self.pid(x, steps)
            GPIO.output(self.stepPin, GPIO.HIGH)
            sleep(delay)
            GPIO.output(self.stepPin, GPIO.LOW)
            sleep(delay)
            #print(delay, "    ", x)
        print("done")
        
    def turn(self, angle):

        m = 2
        zeroAngle = magnetometer1.kalmanReading()
        currentAngle = magnetometer2.kalmanReading()
        targetAngle = zeroAngle[m] + angle

        if (currentAngle[m] - targetAngle) > 0:
            GPIO.output(self.directionPin, GPIO.HIGH)
        else:
            GPIO.output(self.directionPin, GPIO.LOW)

        while abs(currentAngle[m] - targetAngle) > 1:

            if (currentAngle[m] - targetAngle) > 0:
                GPIO.output(self.directionPin, GPIO.HIGH)
            else:
                GPIO.output(self.directionPin, GPIO.LOW)

            pControl = int(abs(currentAngle[m] - targetAngle) * kp)

            print("pControl: {:.2f} \ttarget: {:.2f} \tcurrent: {:.2f}".format(pControl, targetAngle, currentAngle[m]))

            for x in range(pControl):
                GPIO.output(self.stepPin, GPIO.HIGH)
                sleep(delay)
                GPIO.output(self.stepPin, GPIO.LOW)
                sleep(delay)
                # print("stepPin: {:.2f}".format(self.stepPin))

            zeroAngle = magnetometer1.kalmanReading()
            currentAngle = magnetometer2.kalmanReading()
            targetAngle = zeroAngle[m] + angle

    def tiltTurn(self, angle):
        
        m = 30
        delay1 = 0.0001

        currentAngle = magnetometer2.readingX()
        targetAngle = angle

        self.directionCalculation(currentAngle - targetAngle)

        while abs(currentAngle - targetAngle) > 1:

            self.directionCalculation(currentAngle - targetAngle)

            pControl = int(abs(currentAngle - targetAngle) * m)

            print("\tpControl: {:.2f} \tcurrent: {:.2f}".format(pControl, currentAngle))

            for x in range(pControl):
                GPIO.output(self.stepPin, GPIO.HIGH)
                sleep(delay1)
                GPIO.output(self.stepPin, GPIO.LOW)
                sleep(delay1)
                # print("stepPin: {:.2f}".format(self.stepPin))

            currentAngle = magnetometer2.readingX()


class Sensors:
    pass

    # @abstractmethod
    # def reading(self, feature, axis):
    #    pass

    # def filter(self, value):
    #    pass


class Accelerometers(Sensors):
    # MPU6050 Registers and their Address
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    INT_ENABLE = 0x38
    ACCEL_XOUT = 0x3B
    ACCEL_YOUT = 0x3D
    ACCEL_ZOUT = 0x3F
    GYRO_XOUT = 0x43
    GYRO_YOUT = 0x45
    GYRO_ZOUT = 0x47

    bus = smbus.SMBus(1)

    def __init__(self, deviceAddress):

        if deviceAddress == 68:
            deviceAddress = 0x68
        elif deviceAddress == 69:
            deviceAddress = 0x69
        else:
            print("Address is wrong!")

        self.deviceAddress = deviceAddress

        # bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards

        # write to sample rate register
        self.bus.write_byte_data(deviceAddress, self.SMPLRT_DIV, 7)

        # Write to power management register
        self.bus.write_byte_data(deviceAddress, self.PWR_MGMT_1, 1)

        # Write to Configuration register
        self.bus.write_byte_data(deviceAddress, self.CONFIG, 0)

        # Write to Gyro configuration register
        self.bus.write_byte_data(deviceAddress, self.GYRO_CONFIG, 24)

        # Write to interrupt enable register
        self.bus.write_byte_data(deviceAddress, self.INT_ENABLE, 1)

    def read_raw_data(self, registryAddress):

        try:
            # Accelerometer and Gyroscopes value are 16-bit
            high = self.bus.read_byte_data(self.deviceAddress, registryAddress)
            low = self.bus.read_byte_data(self.deviceAddress, registryAddress + 1)

            # concatenate higher and lower value
            value = ((high << 8) | low)

            # to get signed value from mpu6050
            if (value > 32768):
                value = value - 65536
            return value

        except:
            raise

    def tmp(self):
        # Read Accelerometer raw value
        acc_x = self.read_raw_data(self.ACCEL_XOUT)
        acc_y = self.read_raw_data(self.ACCEL_YOUT)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT)

        # Read Gyroscope raw value
        gyro_x = self.read_raw_data(self.GYRO_XOUT)
        gyro_y = self.read_raw_data(self.GYRO_YOUT)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT)

        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0

        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0

        in_min = 1
        in_max = -1
        out_min = -90
        out_max = 90

        # Convert accelerometer Y axis values from 0 to 180
        value = (Ay - 1) * () / (in_max - in_min) + out_min
        value = int(value)
        print(value)

    def readingX(self):
        
        acc_x = []
        
        for i in range(30):
            acc_x = np.append(float(self.read_raw_data(self.ACCEL_XOUT)), acc_x)
        
        
        acc_x_av = np.average(acc_x)
        Ax = acc_x_av / 16384.0
        
        in_min = 1
        in_max = -1
        out_min = -90
        out_max = 90
        
        value = (Ax - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        value = round(value, 2)
        #print(value)
        return value
        

    def reading(self, feature, axis):
        if axis == "x":
            axisValue = 0
        elif axis == "y":
            axisValue = 1
        elif axis == "z":
            axisValue = 2
        else:
            print("Axis does not exist!")
            pass

        in_min = 1
        in_max = -1
        out_min = -90
        out_max = 90

        if feature == "accelerometer":
            Ay = self.read_raw_data(self.ACCEL_YOUT)
            Ay = Ay / 16384.0
            value = (Ay - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
            return value
        elif feature == "gyroscope":
            value = self.read_raw_data(self.GYRO_YOUT)
            return value
        else:
            print("Feature does not exist!")
            pass
    
    def accelerometerReading(self):
        acc_x = self.read_raw_data(self.ACCEL_XOUT)
        acc_y = self.read_raw_data(self.ACCEL_YOUT)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT)
        print("{} \t{} \t{}".format(acc_x, acc_y, acc_z)) 


class Magnetometers(Sensors):
    bus = smbus.SMBus(1)
    np.dataCollected = []

    def __init__(self, deviceAddress):

        if deviceAddress == 68:
            deviceAddress = 0x68
        elif deviceAddress == 69:
            deviceAddress = 0x69
        else:
            print("Address is wrong!")

        imu = MPU9250.MPU9250(smbus.SMBus(1), deviceAddress)
        self.imu = imu
        imu.begin()
        
    def accelerometerReading(self):
        imu = self.imu
        acc_x = self.read_raw_data(self.ACCEL_XOUT)
        acc_y = self.read_raw_data(self.ACCEL_YOUT)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT)
        print("{} \t{} \t{}".format(acc_x, acc_y, acc_z))

    def kalmanReading(self):
        imu = self.imu
        sensorfusion = kalman.Kalman()

        imu.readSensor()
        imu.computeOrientation()
        sensorfusion.roll = imu.roll
        sensorfusion.pitch = imu.pitch
        sensorfusion.yaw = imu.yaw

        count = 0
        currTime = time.time()
        imu.readSensor()
        imu.computeOrientation()
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime

        sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0],
                                                  imu.GyroVals[1], imu.GyroVals[2], \
                                                  imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

        # print("Kalmanroll:{0} \tKalmanPitch:{1} \tKalmanYaw:{2} ".format(round(sensorfusion.roll, 2), round(sensorfusion.pitch, 2), round(sensorfusion.yaw, 2)))

        time.sleep(0.01)

        return round(sensorfusion.roll, 2), round(sensorfusion.pitch, 2), round(sensorfusion.yaw, 2)


class Arm:
    microStepPin = 18
    GPIO.setup(microStepPin, GPIO.OUT)
    GPIO.output(microStepPin, GPIO.HIGH)

    shoulderPan = StepperJoint(15, 14)
    shoulderTilt = StepperJoint(24, 23)
    elbow = ServoJoint(17, 270)
    wristTilt = ServoJoint(27)
    wristPan = ServoJoint(22)

    # j1, j2, j3, j4, j5 = None

    shoulderPan_stepPin = None
    shoulderPan_directionPin = None
    shoulderTilt_stepPin = None
    shoulderTilt_directionPin = None
    elbow_controlPin = None
    elbow_rangeOfMotion = None
    wristTilt_controlPin = None
    wristTilt_rangeOfMotion = None
    wristPan_controlPin = None
    wristPan_rangeOfMotion = None

    def __init__(self, shoulderPan_stepPin, shoulderPan_directionPin,
                 shoulderTilt_stepPin, shoulderTilt_directionPin,
                 elbow_controlPin, elbow_rangeOfMotion,
                 wristTilt_controlPin, wristTilt_rangeOfMotion,
                 wristPan_controlPin, wristPan_rangeOfMotion):
        self.shoulderPan = StepperJoint(shoulderPan_stepPin, shoulderPan_directionPin)
        self.shoulderTilt = StepperJoint(shoulderTilt_stepPin, shoulderTilt_directionPin)
        self.elbow = ServoJoint(elbow_controlPin, elbow_rangeOfMotion)
        self.wristTilt = ServoJoint(wristTilt_controlPin, wristTilt_rangeOfMotion)
        self.wristPan = ServoJoint(wristPan_controlPin, wristPan_rangeOfMotion)
 
 
 
def move(position):
    
    elbow.turn(position[2])
    shoulderPan.manualTurn(position[0])
    elbow.turn(position[2])
    shoulderTilt.tiltTurn(position[1])
    elbow.turn(position[2])
    wristTilt.turn(position[3])
    wristPan.turn(position[4])
    elbow.turn(position[2])

# Instantiate variables


arm = Arm(14, 15, 23, 24, 17, 270, 27, 180, 22, 180)

shoulderPan = arm.shoulderPan
shoulderTilt = arm.shoulderTilt
elbow = arm.elbow
wristTilt = arm.wristTilt
wristPan = arm.wristPan

#magnetometer1 = Magnetometers(68)
magnetometer2 = Accelerometers(69)

elbow.turn(-85)
wristPan.turn(90)
wristTilt.turn(0)

routine1 = [10,10,20,10,10,20,10,-90]

showPosition = [0, -2, -85, 0, 0]
extendedPosition = [60, -2, 0, 0, 0]
fun1 = [-60, -30, -90, 0, 90]


fun2 = [0, -2, -90, 0, 0]
fun3 = [0, -2, -90, 0, 0]


while True:

     move(showPosition)
     sleep(1)
     move(extendedPosition)
     sleep(1)
     move(fun1)
     sleep(1)
     move(fun2)
     sleep(1)
    
    #choice = input('insert an angle for elbow servo: ')
    #shoulderTilt.tiltTurn(int(choice))
    #elbow.manualTurn(int(choice)*-1)
    
    # t=float(input("value? "))
    # shoulderPan.manualTurn(t)
    
    # for i in range(len(routine1)):
    #    shoulderPan.manualTurn(routine1[i])
    #    sleep(1)
    
    