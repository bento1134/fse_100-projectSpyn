#!/usr/bin/env pybricks-micropython
import socket

from pybricks.ev3devices import ColorSensor, Motor, TouchSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

ev3 = EV3Brick()

left_motor = Motor(Port.C)
right_motor = Motor(Port.D)
lift_motor = Motor(Port.A)

estop = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

last_color = None
canidate = None
confidence = 0
STABLE_READINGS = 5


def emergency_stop():
    robot.stop()
    left_motor.brake()  # actively brake instead of coasting
    right_motor.brake()
    # ev3.speaker.beep(frequency=500, duration=50)


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 12345))
server_socket.listen(1)
print("Waiting for connection...")
client_socket, address = server_socket.accept()
print(f"Connected to {address}")
client_socket.settimeout(0.05)  # recv gives up after 50 ms so we can poll the button

buf = ""
try:
    while True:
        # 1. E-stop is checked every loop, before anything else
        if estop.pressed():
            emergency_stop()
            break

        # 2. Read whatever has arrived (or nothing)
        try:
            data = client_socket.recv(2)
        except OSError:
            data = None  # timeout, no data this cycle
        else:
            if not data:  # client disconnected
                break
            buf += data.decode()

        # 3. Process complete 2-char commands
        while len(buf) >= 2:
            key, action = buf[0], buf[1]
            buf = buf[2:]
            if action == "d":
                if key == "w":
                    robot.drive(900, 0)
                elif key == "s":
                    robot.drive(-900, 0)
                elif key == "a":
                    robot.drive(0, -450)
                elif key == "d":
                    robot.drive(0, 450)
                elif key == "l":
                    lift_motor.run_time(200, 100)
            elif action == "u" and key in ("w", "a", "s", "d"):
                robot.stop()

        color = color_sensor.color()

        if color == candidate:
            confidence += 1
        else:
            candidate = color
            count = 1

        # Check if the detected color has a corresponding beep frequency
        if count >= STABLE_READINGS and candidate != last_color:
            if candidate == Color.RED:
                emergency_stop()
                wait(1000)
            elif candidate == Color.BLUE:
                for _ in range(2):
                    ev3.speaker.beep(frequency=500, duration=100)
                    wait(200)
            elif candidate == Color.GREEN:
                for _ in range(3):
                    ev3.speaker.beep(frequency=500, duration=100)
                    wait(200)
            last_color = candidate
finally:
    robot.stop()  # never leave the motors running if we crash or disconnect
    left_motor.brake()
    right_motor.brake()
    client_socket.close()
    server_socket.close()
