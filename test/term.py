import serial
import fileinput
import threading
from time import sleep

import signal

s = serial.Serial('/dev/ttyUSB0', 115200)
running = True


def ctrl_z_handler(signum, frame):
    s.write('\x1A\r\n'.encode('ascii'))


signal.signal(signal.SIGTSTP, ctrl_z_handler)


def transmit_input():
    try:
        for line in fileinput.input():
            if not running:
                break
            s.write((line.strip() + '\r\n').encode('ascii'))
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
