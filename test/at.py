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


for cmd_line in commands:
    cmd = cmd_line.strip()
    cmd_len = len(cmd)
    if cmd_len > 0:
        print("> {0}".format(cmd))
    s.write((cmd + '\r\n').encode('ascii'))
    for i in range(5 if cmd_len > 0 else 1):
        line_b = s.readline()
        print(line_b)
        print(line_b.decode('ascii'))
