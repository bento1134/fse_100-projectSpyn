#!/usr/bin/env pybricks-micropython
import socket

from pybricks.ev3devices import ColorSensor, Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()

left_motor = Motor(Port.C)
right_motor = Motor(Port.D)
lift_motor = Motor(Port.A)

bumper = TouchSensor(Port.S1)
ultra_sensor = UltrasonicSensor(Port.S2)
gyro = GyroSensor(Port.S3)
color_sensor = ColorSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=175)

# --- Maze Tuning Parameters ---
MAZE_SPEED = 150      # mm/s (slower speed for better stability and control)
TARGET_DIST = 170     # mm from right wall
OPEN_DIST = 375       # mm: right side considered open
KP_WALL = 0.4         # Wall-following proportional gain
KP_HEADING = 5.0      # Heading correction gain for driving straight
KI_HEADING = 0.15
TURN_RATE = 35        # Max turn rate (deg/s) - lowered to prevent over-turning

clock = StopWatch()
autopilot_active = False
state = "STRAIGHT"
state_start = 0
turn_target = 0
next_state = "FOLLOW"


def stop_all():
    robot.stop()
    left_motor.brake()
    right_motor.brake()


def set_state(new_state):
    global state, state_start, heading_integral
    state = new_state
    state_start = clock.time()
    heading_integral = 0


def elapsed():
    return clock.time() - state_start


def drive_straight(speed, target_heading):
    global heading_integral
    error = target_heading - gyro.angle()
    heading_integral = max(-100, min(100, heading_integral + error))
    correction = KP_HEADING * error + KI_HEADING * heading_integral
    correction = max(-60, min(60, correction))
    robot.drive(speed, correction)

def turn_to(target, timeout=4000):
    t = StopWatch()
    while t.time() < timeout:
        err = target - gyro.angle()
        print("turn", target, gyro.angle(), err)
        if abs(err) <= 1:
            robot.stop()
            wait(200)
            if abs(target - gyro.angle()) <= 2:
                break

        rate = max(-TURN_RATE, min(TURN_RATE, err * 2.0))
        if 0 < rate < 12:
            rate = 12
        elif -12 < rate < 0:
            rate = -12

        robot.drive(0, rate)
        wait(10)
    robot.stop()


def start_turn(delta, then):
    global turn_target
    turn_target = round((gyro.angle() + delta) / 90) * 90
    turn_to(turn_target)
    if turn_target == 0:
        set_state("STRAIGHT")
    else:
        set_state(then)


def autopilot_start():
    global autopilot_active, turn_target
    gyro.reset_angle(0)
    turn_target = 0
    autopilot_active = True
    set_state("STRAIGHT")


def autopilot_stop():
    global autopilot_active
    autopilot_active = False
    stop_all()


def autopilot_step():
    if state == "STRAIGHT":
        drive_straight(MAZE_SPEED, turn_target)
        if bumper.pressed():
            set_state("BACKUP")

    elif state == "BACKUP":
        robot.drive(-MAZE_SPEED, 0)
        if elapsed() > 700:
            start_turn(-90, "FOLLOW")

    elif state == "FOLLOW":
        if bumper.pressed():
            set_state("BACKUP")
            return
        dist = ultra_sensor.distance()
        if dist > OPEN_DIST:
            set_state("CLEAR_CORNER")
            return
        # Wall following with proportional steering
        wall_error = dist - TARGET_DIST
        steer = max(-30, min(30, KP_WALL * wall_error))
        robot.drive(MAZE_SPEED, steer)

    elif state == "CLEAR_CORNER":
        drive_straight(MAZE_SPEED, turn_target)
        if bumper.pressed():
            set_state("BACKUP")
        elif elapsed() > 1000:
            start_turn(90, "ENTER")

    elif state == "ENTER":
        drive_straight(MAZE_SPEED, turn_target)
        if bumper.pressed():
            set_state("BACKUP")
        elif elapsed() > 1000:
            set_state("FOLLOW")


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 12345))
server_socket.listen(1)
print("Waiting for connection...")
client_socket, address = server_socket.accept()
client_socket.settimeout(0.05)

buf = ""
try:
    while True:
        try:
            data = client_socket.recv(2)
        except OSError:
            data = None
        else:
            if not data:
                break
            buf += data.decode()

        while len(buf) >= 2:
            key, action = buf[0], buf[1]
            buf = buf[2:]
            if action == "d":
                if key == "p":
                    autopilot_start()
                elif key == "x":
                    autopilot_stop()
                elif not autopilot_active:
                    if key == "w":
                        robot.drive(900, 0)
                    elif key == "s":
                        robot.drive(-900, 0)
                    elif key == "a":
                        robot.drive(0, -450)
                    elif key == "d":
                        robot.drive(0, 450)
                    elif key == "l":
                        lift_motor.run(200)
            elif action == "u" and key in ("w", "a", "s", "d", "l"):
                if not autopilot_active:
                    robot.stop()
                    lift_motor.brake()

        if autopilot_active:
            autopilot_step()

        # Color sensor check can be re-enabled here if needed
        
finally:
    stop_all()
    client_socket.close()
    server_socket.close()
