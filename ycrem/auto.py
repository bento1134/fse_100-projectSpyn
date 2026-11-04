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

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# --- color debounce ---
last_color = None
candidate = None
confidence = 0
STABLE_READINGS = 8

# --- maze tuning ---
MAZE_SPEED = 250      # mm/s
TARGET_DIST = 170     # mm from right wall
OPEN_DIST = 375       # mm: right side considered open past this
KP = 0.0              # deg/s per mm of error
TURN_RATE = 60        # deg/s
HEADING_TOL = 4       # deg

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
    global state, state_start
    state = new_state
    state_start = clock.time()


def elapsed():
    return clock.time() - state_start

def turn_to(target):
    """Blocking gyro turn with slow-down near target and brake."""
    t = StopWatch()
    while t.time() < 3000:                       # safety timeout
        err = target - gyro.angle()
        if abs(err) <= 2:
            robot.stop()
            left_motor.brake()
            right_motor.brake()
            wait(150)                            # let it settle
            if abs(target - gyro.angle()) <= 3:
                return                           # truly there
            continue                             # drifted past, correct it
        rate = max(25, min(TURN_RATE, abs(err) * 2.5))
        robot.drive(0, rate if err > 0 else -rate)
        wait(5)
    robot.stop()

def start_turn(delta, then):
    global turn_target
    # snap to a multiple of 90 so errors don't accumulate across turns
    turn_target = round((gyro.angle() + delta) / 90) * 90
    turn_to(turn_target)
    if turn_target == 0:
        set_state("STRAIGHT")                    # Pledge counter back to 0
    else:
        set_state(then)


def autopilot_start():
    global autopilot_active
    gyro.reset_angle(0)                  # original heading = 0
    autopilot_active = True
    set_state("STRAIGHT")


def autopilot_stop():
    global autopilot_active
    autopilot_active = False
    stop_all()


def autopilot_step():
    heading = gyro.angle()

    if state == "STRAIGHT":
        # Pledge: counter is 0, ignore walls and go straight until blocked
        robot.drive(MAZE_SPEED, 0)
        if bumper.pressed():
            set_state("BACKUP")

    elif state == "BACKUP":
        robot.drive(-MAZE_SPEED, 0)
        if elapsed() > 600:
            start_turn(-90, "FOLLOW")    # turn left so wall is on the right

    elif state == "TURN":
        err = turn_target - heading
        if abs(err) <= HEADING_TOL:
            robot.stop()
            if abs(heading) <= HEADING_TOL:
                set_state("STRAIGHT")    # counter back to 0
            else:
                set_state(next_state)
        else:
            robot.drive(0, TURN_RATE if err > 0 else -TURN_RATE)

    elif state == "FOLLOW":
        if bumper.pressed():
            set_state("BACKUP")
            return
        dist = ultra_sensor.distance()
        if dist > OPEN_DIST:
            set_state("CLEAR_CORNER")    # opening on the right
            return
        steer = max(-40, min(40, KP * (dist - TARGET_DIST)))
        robot.drive(MAZE_SPEED, steer)

    elif state == "CLEAR_CORNER":
        robot.drive(MAZE_SPEED, 0)
        if bumper.pressed():
            set_state("BACKUP")
        elif elapsed() > 1000:
            start_turn(90, "ENTER")      # turn right into the opening

    elif state == "ENTER":
        robot.drive(MAZE_SPEED, 0)
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
                elif not autopilot_active:   # manual keys ignored in maze mode
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

        color = color_sensor.color()
        if color == candidate:
            confidence += 1
        else:
            candidate = color
            confidence = 1

            #        if confidence >= STABLE_READINGS and candidate != last_color:
                    #if candidate == Color.RED:
                    #autopilot_stop()         # red tile = goal / stop
                    #ev3.speaker.beep(frequency=500, duration=300)
                #elif candidate == Color.BLUE:
                #ev3.speaker.beep(frequency=500, duration=100)
            #elif candidate == Color.GREEN:
            #ev3.speaker.beep(frequency=800, duration=100)
            #last_color = candidate
finally:
    stop_all()
    client_socket.close()
    server_socket.close()
