import serial

PORT = '/dev/ttyUSB0'

MAX = 0x7D0
CENTER = 0x5DC
MIN = 0x3E8


def tank_convert(steering, throttle):
    left = throttle + steering - CENTER
    if left > MAX:
        left = MAX
    elif left < MIN:
        left = MIN
    right = throttle + CENTER - steering
    if right > MAX:
        right = MAX
    elif right < MIN:
        right = MIN
    return left, right


with serial.Serial(PORT, 115200, timeout=0.002, inter_byte_timeout=0.001) as port:
    errcnt = 0
    stop = True
    while True:
        rx = port.readline()
        if (len(rx) == 32) and (rx[0] == 0x20) and (rx[1] == 0x40):
            errcnt = 0
            if stop:
                print('START')
                stop = False
            ch = []
            for i in range(10):
                ch.append((rx[2*i+3] << 8) | rx[2*i+2])
            for c in ch:
                print("{:04X} ".format(c), end='')
            left, right = tank_convert(ch[0], ch[1])
            print('{:04X} {:04X}'.format(left, right))
        else:
            if not stop:
                errcnt += 1
                if errcnt > 10:
                    print('STOP')
                    stop = True
