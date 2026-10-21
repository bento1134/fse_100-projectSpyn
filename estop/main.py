#Group 4
#Bennett Rodriguez, Joshua Adame, Roberto Kanygin, Robin Bui

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.ev3devices import TouchSensor

ev3 = EV3Brick()
touch_sensor = TouchSensor(Port.S1)

estop = False

while not touch_sensor.pressed():
    ev3.speaker.beep()

