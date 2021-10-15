import serial
import fileinput
import threading
from time import sleep

s = serial.Serial('/dev/ttyUSB0', 115200)
running = True


def transmit_input():
    try:
        for line in fileinput.input():
            if not running:
                break
            s.write(line.encode('ascii'))
    except KeyboardInterrupt:
        pass


threading.Thread(target=transmit_input).start()

b = []
while running:
    try:
        print(s.readline().decode('ascii').strip())
        # print(s.read())
    except KeyboardInterrupt:
        s.close()
        running = False
