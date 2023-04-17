from abc import ABC, abstractmethod
import RPi.GPIO as GPIO
import pigpio
import smbus2 as smbus
import time
from time import sleep
import numpy as np

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pi = pigpio.pi()

delay = 0.0005
kp = 20


class Joint:
    @abstractmethod
    def turn(self, angle):
        pass


class ServoJoint(Joint):
    controlPin = 0
    rangeOfMotion = 0

    def __init__(self, controlPin, rangeOfMotion=180):
        self.controlPin = controlPin
        self.rangeOfMotion = rangeOfMotion

    def turn(self, angle):
        frequency = self.angleToFrequency(angle)
        pi.set_servo_pulsewidth(self.controlPin, frequency)

    def angleToFrequency(self, angle):
        if self.rangeOfMotion == 270:
            return (1000/135)*angle + 1500
        else:
            return (1000/90)*angle + 1500


class StepperJoint(Joint):
    stepPin = 0
    directionPin = 0

    def __init__(self, stepPin, directionPin):
        self.stepPin = stepPin
        self.directionPin = directionPin
        GPIO.setup(stepPin, GPIO.OUT)
        GPIO.setup(directionPin, GPIO.OUT)

    def turn(self, targetAngle):

        currentAngle = magnetometer1.reading("accelerometer1", "y")

        while abs(currentAngle - targetAngle) < 1:

            pControl = currentAngle - targetAngle

            for x in range(pControl):
                GPIO.output(self.stepPin, GPIO.HIGH)
                sleep(delay)
                GPIO.output(self.stepPin, GPIO.LOW)
                sleep(delay)
            pass


class Sensors:

    @abstractmethod
    def reading(self, feature, axis):
        pass


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
        elif feature == "gyroscope":
            value = self.read_raw_data(self.GYRO_YOUT)
        else:
            print("Feature does not exist!")
            pass

        return value


class Magnetometers(Sensors):

    def __init__(self, address):

        if address == 68:
            address = MPU9050_ADDRESS_68
        elif address == 69:
            address = MPU9050_ADDRESS_69
        else:
            print("Address is wrong!")
            pass

        self.mpu = MPU9250(
                    address_ak=AK8963_ADDRESS,
                    address_mpu_master=address,
                    address_mpu_slave=None,
                    bus=1,
                    gfs=GFS_1000,
                    afs=AFS_8G,
                    mfs=AK8963_BIT_16,
                    mode=AK8963_MODE_C100HZ)

        self.mpu.configure()  # Apply the settings to the registers.

        # print("|.....MPU9250 in 0x68 Address.....|")

    def reading(self, feature, axis):

        axisValue = None
        value = None

        if axis == "x":
            axisValue = 0
        elif axis == "y":
            axisValue = 1
        elif axis == "z":
            axisValue = 2
        else:
            print("Axis does not exist!")
            pass

        if feature == "accelerometer":
            value = self.mpu.readAccelerometerMaster(axis)
        elif feature == "gyroscope":
            value = self.mpu.readGyroscopeMaster()
        elif feature == "magnetometer":
            value = self.mpu.readMagnetometerMaster()
        elif feature == "temperature":
            value = self.mpu.readTemperatureMaster()
        else:
            print("Feature does not exist!")
            pass

        return value[axisValue]

    sleep(1)


class Arm:

    microStepPin = 18
    GPIO.setup(microStepPin, GPIO.OUT)
    GPIO.output(microStepPin, GPIO.HIGH)

    shoulderPan = StepperJoint(5, 6)
    shoulderTilt = StepperJoint(7, 8)
    elbow = ServoJoint(23, 270)
    wristTilt = ServoJoint(24)
    wristPan = ServoJoint(25)

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

    def __init__(self,  shoulderPan_stepPin, shoulderPan_directionPin,
                        shoulderTilt_stepPin, shoulderTilt_directionPin,
                        elbow_controlPin, elbow_rangeOfMotion,
                        wristTilt_controlPin, wristTilt_rangeOfMotion,
                        wristPan_controlPin, wristPan_rangeOfMotion):

        self.shoulderPan = StepperJoint(shoulderPan_stepPin, shoulderPan_directionPin)
        self.shoulderTilt = StepperJoint(shoulderTilt_stepPin, shoulderTilt_directionPin)
        self.elbow = ServoJoint(elbow_controlPin, elbow_rangeOfMotion)
        self.wristTilt = ServoJoint(wristTilt_controlPin, wristTilt_rangeOfMotion)
        self.wristPan = ServoJoint(wristPan_controlPin, wristPan_rangeOfMotion)

    def inverseKinematics(self):

        pass

# Instantiate variables


arm = Arm(14, 15, 23, 24, 25, 270, 8, 180, 4, 180)


magnetometer1 = Magnetometers(68)
magnetometer2 = Magnetometers(69)

while True:

    magnetometer1Angle = magnetometer1.reading("magnetometer", "x")
    magnetometer2Angle = magnetometer2.reading("magnetometer", "y")

    print("Accelerometer 1: {:.2f} \tMagnetometer 1: {:.2f}".format(magnetometer1Angle, magnetometer2Angle))

    sleep(0.1)
