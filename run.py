import serial,time
import RPi.GPIO as GPIO


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

    time.sleep(0.5)

    param = ser.read(4)

    print(param)

    ser.close()
    GPIO.cleanup()