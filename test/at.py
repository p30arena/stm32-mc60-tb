from typing import Tuple
import serial
from time import sleep
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
running = True
commands = []


with open(os.path.join(dir_path, "at_commands.txt"), "r") as f:
    for line in f.readlines():
        commands.append(line)


def readline() -> Tuple[bool, str]:
    ok = True
    line = s.readline().decode('ascii').strip()
    if line == "ERROR" or line.startswith("+CME ERROR"):
        ok = False
    print(line)
    return ok, line


def writeline(cmd='') -> None:
    s.write((cmd + '\r\n').encode('ascii'))


for line in commands:
    code, tries, cmd = line.strip().split(':')
    tries = int(tries)

    if code == -1:
        print("> {0} for {1}s".format(cmd, tries))
        sleep(tries)
        continue

    print("> {0}".format(cmd))

    writeline(cmd + '\r\n')

    if tries == -1:
        readline()
        writeline()
        readline()
    elif tries == -2:
        readline()
        writeline('\x1A') # ctrl + z
        readline()
        readline()
        readline()
    else:
        for i in range(tries):
            readline()
