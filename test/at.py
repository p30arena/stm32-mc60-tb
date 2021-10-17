import serial
import fileinput
import threading
from time import sleep
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
running = True
commands = []


with open(os.path.join(dir_path, "at_commands.txt"), "r") as f:
    for line in f.readlines():
        commands.append(line)


for cmd in commands:
    print("> {0}".format(cmd.strip()))
    s.write(cmd.encode('ascii'))
    for i in range(5):
        print(s.readline().decode('ascii'))
