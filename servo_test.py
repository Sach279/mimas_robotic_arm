import pigpio

pi = pigpio.pi()

pin = 17
frequency  = 833

while True:
    pi.set_servo_pulsewidth(pin, frequency)