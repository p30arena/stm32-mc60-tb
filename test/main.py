import serial

s = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)

while True:
    try:
        print(s.readline().decode('ascii'))
    except KeyboardInterrupt:
        s.close()
        break
