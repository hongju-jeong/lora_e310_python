import serial, time, datetime
ser_data=''

ser=serial.Serial('/dev/ttyAMA0', 115200)
print(ser.name)

counter=0
timestamp = datetime.datetime.now().timestamp()
while(1):
    if(datetime.datetime.now().timestamp() - timestamp > 1):
        print(counter)
        timestamp = datetime.datetime.now().timestamp()
        counter = 0
    #time.sleep(0.03)
    ser_data = ser.read(5)   
    #print(ser_data)
    f=open('test.jpg','ab')
    f.write(ser_data)
    f.close()
    counter += 1
    #ser_data.strip(b'\r').strip(b'\n')   
    #print(ser_data)
    if b'test' in ser_data:
        print("test received")
        ser.close()
        break

print("end")
