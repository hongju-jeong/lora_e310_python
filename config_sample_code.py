import serial,time
import RPi.GPIO as GPIO

"""
bitrate : 115200
Air rate : 125k
"""

if __name__ == "__main__":
    m0_pin_number = 23
    m1_pin_number = 24

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(m0_pin_number, GPIO.OUT)
    GPIO.setup(m1_pin_number, GPIO.OUT)

    GPIO.output(m0_pin_number,GPIO.LOW)
    GPIO.output(m1_pin_number,GPIO.HIGH)

    ser = serial.Serial(port='/dev/ttyAMA0',baudrate = 9600)
    
    ser.write(bytes(b'\xc1\x00\x01'))
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(param)
    
    ser.write(bytes(b'\xc1\x01\x01'))
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(param)
    
    ser.write(bytes(b'\xc1\x05\x01'))
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(850.125 + param[3]*0.2)
    
    ser.write(bytes(b'\xc0\x05\x01\x41'))
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(900 + param[3]*0.2)
    
    ser.write(bytes(b'\xc1\x06\x01'))
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(bin(param[3]))
    
    send_buff = bytearray([int('0xc0',16),int('0x06',16),int('0x01',16), int('0b11',2)])
    ser.write(send_buff)
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(bin(param[3]))
    
    
    send_buff[0] = int('0xc1',16)
    send_buff[1] = int('0x03',16)
    send_buff[2] = int('0x01',16)
    send_buff.pop(3)
    ser.write(send_buff)
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(bin(param[3]))
    
    send_buff[0] = int('0xc0',16)
    send_buff[1] = int('0x03',16)
    send_buff[2] = int('0x01',16)
    send_buff.append(int('0b11100111',2))
    
    ser.write(send_buff)
    time.sleep(0.1)
    param = bytearray(ser.read(4))
    print(bin(param[3]))
    
    
    

    ser.close()
    GPIO.cleanup()


