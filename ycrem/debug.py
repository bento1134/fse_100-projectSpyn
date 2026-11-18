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
MAZE_SPEED = 150      # mm/s
TARGET_DIST = 170     # mm from right wall
OPEN_DIST = 375       # mm: right side considered open
KP_WALL = 0.4         # Wall-following proportional gain
KP_HEADING = 10.0     # Heading correction gain for driving straight
HEADING_MAX = 60      # Clamp on heading correction (deg/s)
TURN_RATE = 35        # Max turn rate (deg/s)
BACKUP_TIME = 1400    # ms of reversing after a bumper hit
BUMPER_GRACE = 500    # ms after entering FOLLOW before the bumper can trigger BACKUP
WALL_ANGLE_GAIN = 0.15   # degrees of heading offset per mm of wall error
WALL_ANGLE_MAX = 15      # max degrees off the corridor heading
# --- Debug Settings ---
DEBUG = True          # master switch for all prints
LOOP_LOG_MS = 250     # how often the state-loop logs repeat (ms)
TURN_LOG_EVERY = 1    # log every Nth iteration of turn_to (raise if prints slow the turn)

clock = StopWatch()
autopilot_active = False
state = "STRAIGHT"
state_start = 0
turn_target = 0
last_log_time = 0


# ---------------------------------------------------------------- logging
def log(tag, msg=""):
    if DEBUG:
        print("[{:>7}] {:<8} {}".format(clock.time(), tag, msg))


def log_due():
    """True at most once per LOOP_LOG_MS, so repeating states don't flood the console."""
    global last_log_time
    if not DEBUG:
        return False
    now = clock.time()
    if now - last_log_time >= LOOP_LOG_MS:
        last_log_time = now
        return True
    return False


def sensors():
    return "gyro={} dist={} bump={}".format(
        gyro.angle(), ultra_sensor.distance(), bumper.pressed())


# ---------------------------------------------------------------- helpers
def stop_all():
    log("STOP", "stop_all()")
    robot.stop()
    left_motor.brake()
    right_motor.brake()


def set_state(new_state, reason=""):
    global state, state_start
    if DEBUG:
        log("STATE", "{} -> {} ({}) {}".format(state, new_state, reason, sensors()))
    state = new_state
    state_start = clock.time()


def elapsed():
    return clock.time() - state_start


def drive_straight(speed, target_heading):
    """Drive straight while correcting heading using the gyro."""
    angle = gyro.angle()
    error = target_heading - angle
    correction = max(-HEADING_MAX, min(HEADING_MAX, KP_HEADING * error))
    robot.drive(speed, correction)
    if log_due():
        log(state, "drive_straight target={} angle={} err={} corr={}".format(
            target_heading, angle, error, correction))


    """Gyro turn with slow-down near the target. Logs every step plus a summary."""
def turn_to(target, timeout=None):
    if timeout is None:
        timeout = abs(target - gyro.angle()) * 60 + 1500   # ms, about 7 s for 90 degrees
    t = StopWatch()
    start_angle = gyro.angle()
    direction = 1 if target > start_angle else -1
    peak_overshoot = 0
    iterations = 0
    last_angle = start_angle
    last_change = 0
    reached = False

    log("TURN", "begin start_angle={} target={} delta={}".format(
        start_angle, target, target - start_angle))

    while t.time() < timeout:
        angle = gyro.angle()
        err = target - angle
        iterations += 1

        # Degrees past the target in the direction of travel (positive = overshot)
        overshoot = (angle - target) * direction
        if overshoot > peak_overshoot:
            peak_overshoot = overshoot

        # Detect a gyro that isn't changing while we're commanding a turn
        if angle != last_angle:
            last_angle = angle
            last_change = t.time()
        elif t.time() - last_change > 500:
            log("TURN", "WARN gyro unchanged for 500ms at angle={} (held/stuck/slipping?)".format(angle))
            last_change = t.time()

        if abs(err) <= 1:
            robot.stop()
            log("TURN", "within +-1: angle={} err={} overshoot={}, stopping and settling".format(
                angle, err, overshoot))
            wait(200)
            settled = gyro.angle()
            log("TURN", "settled angle={} (moved {} after stop)".format(settled, settled - angle))
            settled_over = (settled - target) * direction
            if settled_over > peak_overshoot:
                peak_overshoot = settled_over
            last_angle = settled
            last_change = t.time()
            if abs(target - settled) <= 2:
                reached = True
                break
            log("TURN", "outside +-2 after settle, correcting again")
            continue

        rate = err * 2.0
        if rate > TURN_RATE:
            rate = TURN_RATE
        elif rate < -TURN_RATE:
            rate = -TURN_RATE
        if 0 < rate < 12:
            rate = 12
        elif -12 < rate < 0:
            rate = -12

        robot.drive(0, rate)
        if iterations % TURN_LOG_EVERY == 0:
            log("TURN", "t={} angle={} err={} rate={}".format(t.time(), angle, err, rate))
        wait(10)

    robot.stop()
    final = gyro.angle()
    if reached:
        log("TURN", "DONE target={} final={} peak_overshoot={} iterations={} time={}ms".format(
            target, final, peak_overshoot, iterations, t.time()))
    else:
        log("TURN", "TIMEOUT target={} final={} peak_overshoot={} iterations={} time={}ms".format(
            target, final, peak_overshoot, iterations, t.time()))


def start_turn(delta, then):
    global turn_target
    before = gyro.angle()
    turn_target = round((before + delta) / 90) * 90
    log("TURN", "start_turn delta={} gyro_now={} -> turn_target={} then={}".format(
        delta, before, turn_target, then))
    turn_to(turn_target)
    if turn_target == 0:
        set_state("STRAIGHT", "turn complete, heading is 0")
    else:
        set_state(then, "turn complete")


def autopilot_start():
    global autopilot_active, turn_target
    log("AUTO", "START - resetting gyro (was {})".format(gyro.angle()))
    gyro.reset_angle(0)
    turn_target = 0
    autopilot_active = True
    set_state("STRAIGHT", "autopilot start")


def autopilot_stop():
    global autopilot_active
    log("AUTO", "STOP requested")
    autopilot_active = False
    stop_all()


def autopilot_step():
    if state == "STRAIGHT":
        drive_straight(MAZE_SPEED, turn_target)
        if bumper.pressed():
            set_state("BACKUP", "bumper pressed in STRAIGHT")

    elif state == "BACKUP":
        robot.drive(-MAZE_SPEED, 0)
        if log_due():
            log("BACKUP", "elapsed={}/{} {}".format(elapsed(), BACKUP_TIME, sensors()))
        if elapsed() > BACKUP_TIME:
            log("BACKUP", "done reversing, turning left")
            start_turn(-90, "FOLLOW")

    elif state == "FOLLOW":
        if bumper.pressed():
            if elapsed() > BUMPER_GRACE:
                set_state("BACKUP", "bumper pressed in FOLLOW")
                return
            elif log_due():
                log("FOLLOW", "bumper pressed but ignored (grace, elapsed={})".format(elapsed()))
        dist = ultra_sensor.distance()
        if dist > OPEN_DIST:
            set_state("CLEAR_CORNER", "dist={} > OPEN_DIST={}".format(dist, OPEN_DIST))
            return
        wall_error = dist - TARGET_DIST
        offset = max(-WALL_ANGLE_MAX, min(WALL_ANGLE_MAX, WALL_ANGLE_GAIN * wall_error))
        if log_due():
            log("FOLLOW", "dist={} wall_err={} offset={} gyro={}".format(
                dist, wall_error, offset, gyro.angle()))
        drive_straight(MAZE_SPEED, turn_target + offset)

    elif state == "CLEAR_CORNER":
        drive_straight(MAZE_SPEED, turn_target)
        if bumper.pressed():
            set_state("BACKUP", "bumper pressed in CLEAR_CORNER")
        elif elapsed() > 1000:
            log("CLEAR", "cleared corner, turning right")
            start_turn(90, "ENTER")

    elif state == "ENTER":
        drive_straight(MAZE_SPEED, turn_target)
        if bumper.pressed():
            set_state("BACKUP", "bumper pressed in ENTER")
        elif elapsed() > 1000:
            set_state("FOLLOW", "entered new corridor")


# ---------------------------------------------------------------- main
log("INIT", "debug logging on; battery={}mV".format(ev3.battery.voltage()))
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 12345))
server_socket.listen(1)
log("SOCK", "Waiting for connection on port 12345...")
client_socket, address = server_socket.accept()
client_socket.settimeout(0.01)
log("SOCK", "Connected: {}".format(address))

buf = ""
try:
    while True:
        try:
            data = client_socket.recv(2)
        except OSError as e:
            data = None
            # Timeouts are expected; only log other socket errors
            if not e.args or e.args[0] not in (11, 110):
                log("SOCK", "recv error: {}".format(e))
        else:
            if not data:
                log("SOCK", "client disconnected")
                break
            buf += data.decode()

        while len(buf) >= 2:
            key, action = buf[0], buf[1]
            buf = buf[2:]
            log("RX", "key={} action={} autopilot={}".format(key, action, autopilot_active))
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
                else:
                    log("RX", "manual key {} ignored (autopilot active)".format(key))
            elif action == "u" and key in ("w", "a", "s", "d", "l"):
                if not autopilot_active:
                    robot.stop()
                    lift_motor.brake()

        if autopilot_active:
            autopilot_step()

        # Color sensor check can be re-enabled here if needed

except Exception as e:
    log("ERROR", repr(e))
    raise
finally:
    log("EXIT", "shutting down")
    stop_all()
    client_socket.close()
    server_socket.close()
