import serial
import time

tty = "/dev/ttyACM0"
baudrate = 115200

ser = serial.Serial(tty, baudrate, timeout=1)

if ser.is_open:
    print(f"Serial port open: {ser.is_open}")
    ser.write('k\n'.encode())
    response = ser.read().decode()
    print(response)
    print('code finished')
   
    
    # time.sleep(10)
    # ser.write('open\n'.encode())
    # time.sleep(10)
    # ser.write('close\n'.encode())