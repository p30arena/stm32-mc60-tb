import serial
import fileinput
import threading

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
running = True
get_input = False


def transmit_input():
    global get_input
    try:
        for line in fileinput.input():
            if not running:
                break
            if len(line.strip()) == 0:
                get_input = True
            else:
                s.write(line.encode('ascii'))
                get_input = False
    except KeyboardInterrupt:
        pass


threading.Thread(target=transmit_input).start()

b = []
while running:
    try:
        if not get_input and len(b) > 0:
            for line in b:
                print(line)
            b.clear()
        line = s.readline().decode('ascii').strip()
        if not get_input:
            print(line)
        else:
            b.append(line)
    except KeyboardInterrupt:
        s.close()
        running = False
