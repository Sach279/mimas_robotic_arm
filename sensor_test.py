import smbus as smbus
import numpy as np

class Accelerometers():
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
        
        for i in range(100):
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


acc1 = Accelerometers(69)

while True:
    
    print(acc1.readingX())

