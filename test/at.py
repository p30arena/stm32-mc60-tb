from typing import Tuple
import serial
from time import sleep
import os
from threading import Thread

dir_path = os.path.dirname(os.path.realpath(__file__))
running = True
commands = []
polled_lines = []

with serial.Serial('/dev/ttyUSB0', 115200) as s:
    def poll_read():
        while running:
            line = s.readline().decode('ascii').strip()
            polled_lines.insert(0, line)

    Thread(target=poll_read).start()

    with open(os.path.join(dir_path, "at_commands.txt"), "r") as f:
        for line in f.readlines():
            commands.append(line)

    def readline() -> Tuple[bool, str]:
        t_wait = 0
        while len(polled_lines) == 0:
            t_wait += 0.5
            sleep(0.5)
            if t_wait > 3:
                raise TimeoutError

        ok = True
        line = polled_lines.pop()
        print(line)
        if line == "ERROR" or line.startswith("+CME ERROR"):
            ok = False
        return ok, line

    def writeline(cmd='') -> None:
        s.write((cmd + '\r\n').encode('ascii'))

    for line in commands:
        code, tries, cmd = line.strip().split('<>')
        code = int(code)
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
            writeline('\x1A')  # ctrl + z
            readline()
            readline()
            readline()
        else:
            while True:
                try:
                    ok, line = readline()
                    if not ok:
                        break
                except TimeoutError:
                    break

    try:
        readline()
    except KeyboardInterrupt:
        pass

    running = False
