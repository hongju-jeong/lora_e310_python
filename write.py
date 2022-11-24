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

str = "emptyarray"
send_buff = ""
for i in range(18):
    send_buff += str
send_buff += "emptyarra"
print(len(send_buff))
counter=0
timestamp = datetime.datetime.now().timestamp()
while 1:
    if(datetime.datetime.now().timestamp() - timestamp > 1):
        print(counter)
        timestamp = datetime.datetime.now().timestamp()
        counter = 0
    
    ser.write(bytes(send_buff+'\n', 'utf-8'))
    ser.flush()
    counter += 1
    

ser.close()
GPIO.cleanup()
