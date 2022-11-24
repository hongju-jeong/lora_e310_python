import serial,time
import RPi.GPIO as GPIO

def pin_wait_delay():
    raise NotImplementedError

ser = serial.Serial(port='/dev/ttyAMA0',baudrate = 9600)
ser.write(bytearray(b'\xc1\x00\x01'))

time.sleep(0.5)

param = ser.read(4)

print(param)


