#!/usr/bini/env python

import odroid_wiringpi as wpi
import time

cl = 31
da = 30
print('before')
wpi.wiringPiSetup()
print('after')
wpi.pinMode(da, 0)
wpi.pinMode(cl, 0)
print('ok pinmode')
counter = 0
clkLastState = wpi.digitalRead(cl)

while True:
    clkState = wpi.digitalRead(cl)
    daState = wpi.digitalRead(da)
    # print(clkState,daState)
    if clkState != clkLastState:
        if daState != clkState:
            counter += 1
        else:
            counter -= 1
        print(counter)
    clkLastState = clkState
    # time.sleep(0.1)
