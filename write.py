import time
import serial
import datetime

ser = serial.Serial(

    port='/dev/ttyAMA0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
counter=0
timestamp = datetime.datetime.now().timestamp()
while 1:
    if(datetime.datetime.now().timestamp() - timestamp > 1):
        print(counter)
        timestamp = datetime.datetime.now().timestamp()
        counter = 0
    now=datetime.datetime.now()
    a=now.strftime("%S")
    b=now.strftime("%M")
    c=now.strftime("%H")
    #print(a)
    ser.write(bytes( c+b+a+'\n','utf-8'))
    ser.flush()
    counter += 1
    #print("sent on",now.strftime("%H:%M:%S:%f"))
    #time.sleep(0.03)


