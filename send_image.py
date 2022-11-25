import time
import serial
import datetime
import RPi.GPIO as GPIO


m0_pin_number = 23
m1_pin_number = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(m0_pin_number, GPIO.OUT)
GPIO.setup(m1_pin_number, GPIO.OUT)

GPIO.output(m0_pin_number,GPIO.LOW)
GPIO.output(m1_pin_number,GPIO.LOW)

ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate = 115200
)

f = open("test.jpg", 'rb')

while 1:
    send_buff = f.read(188)
    if len(send_buff) <= 0:
        break    
    ser.write(bytes(send_buff))
    ser.flush()
    
    time.sleep(0.015)
    

ser.close()
GPIO.cleanup()
