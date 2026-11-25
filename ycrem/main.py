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
MAZE_SPEED = 115      # mm/s
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
ENTER_MAX_MS = 20000
TURN_TOL = 2

# --- Color Actions (edit this table to change what each color does) ---
# Colors:  Color.RED, Color.BLUE, Color.GREEN, Color.YELLOW, Color.BROWN, Color.WHITE, Color.BLACK
# Actions: "PAUSE"  - stop for PAUSE_MS, then carry on
#          "MANUAL" - turn autopilot off and give control to the driver (send 'p' to resume)
#          "FINISH" - end of the maze: turn autopilot off and stop the robot
#                     (does nothing until you press 'o' to arm it; fires once)
# Remove a line to ignore that color; add a line to give another color an action.
COLOR_ACTIONS = [
    (Color.RED,   "PAUSE"),
    (Color.BLACK,  "MANUAL"),
    (Color.GREEN, "FINISH")
]
PAUSE_MS = 1000       # how long a "PAUSE" color stops the robot
STABLE_READINGS = 12   # consecutive identical readings before a color counts

# --- Heading Snap (re-aligns the gyro to the maze walls) ---
SNAP_WINDOW_MS = 1500 # wall distance must stay steady this long ...
SNAP_DIST_TOL = 12    # ... within this many mm (robot is parallel to the wall)
SNAP_MIN_ERR = 2      # deg: ignore heading errors smaller than this
SNAP_MAX_ERR = 15     # deg: don't snap if the heading is further off than this

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

# Color detection state
last_color = None     # last confirmed color (actions only fire when this changes)
candidate = None      # color currently being confirmed
confidence = 0        # consecutive readings of candidate
in_turn = False       # True while turn_to is running; colors are ignored then
finish_armed = False  # FINISH does nothing until the driver sends 'o'

# Heading snap state
heading_offset = 0    # added to the gyro reading so heading() matches the maze walls
wall_samples = []     # recent (time_ms, dist) readings while following a wall
last_wall_sample = 0


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
        heading(), ultra_sensor.distance(), bumper.pressed())


# ---------------------------------------------------------------- helpers
def heading():
    """Gyro angle plus the snap correction. Use this instead of gyro.angle()."""
    return gyro.angle() + heading_offset


def stop_all():
    log("STOP", "stop_all()")
    robot.stop()
    left_motor.brake()
    right_motor.brake()


def set_state(new_state, reason=""):
    global state, state_start, wall_samples
    if DEBUG:
        log("STATE", "{} -> {} ({}) {}".format(state, new_state, reason, sensors()))
    state = new_state
    state_start = clock.time()
    wall_samples = []     # a new state starts a fresh steady-wall measurement


def elapsed():
    return clock.time() - state_start


def try_snap(dist):
    """Re-align the gyro to the maze while following a wall.

    If the right-wall distance has stayed steady for SNAP_WINDOW_MS, the robot is
    running parallel to that wall, so its true heading is turn_target (a multiple
    of 90). Shift the gyro frame (software offset, no sensor reset) to match.
    """
    global heading_offset, wall_samples, last_wall_sample
    now = clock.time()
    if now - last_wall_sample < 50:
        return
    last_wall_sample = now

    wall_samples.append((now, dist))
    while now - wall_samples[0][0] > SNAP_WINDOW_MS:
        wall_samples.pop(0)
    if now - wall_samples[0][0] < SNAP_WINDOW_MS * 0.8:
        return          # not enough history yet

    dists = [s[1] for s in wall_samples]
    if max(dists) - min(dists) > SNAP_DIST_TOL:
        return          # distance is changing: not parallel to the wall

    err = turn_target - heading()
    if abs(err) < SNAP_MIN_ERR or abs(err) > SNAP_MAX_ERR:
        return          # already aligned, or too far off to trust

    before = heading()
    heading_offset += err
    log("SNAP", "wall steady ({}-{}mm): heading {} -> {} (turn_target {})".format(
        min(dists), max(dists), before, heading(), turn_target))
    wall_samples = []


def drive_straight(speed, target_heading):
    """Drive straight while correcting heading using the gyro."""
    angle = heading()
    error = target_heading - angle
    correction = max(-HEADING_MAX, min(HEADING_MAX, KP_HEADING * error))
    robot.drive(speed, correction)
    if log_due():
        log(state, "drive_straight target={} angle={} err={} corr={}".format(
            target_heading, angle, error, correction))


def turn_to(target, timeout=None):
    """Gyro turn with slow-down near the target. Logs every step plus a summary."""
    global in_turn
    in_turn = True
    if timeout is None:
        timeout = abs(target - heading()) * 60 + 1500
    t = StopWatch()
    start_angle = heading()
    direction = 1 if target > start_angle else -1
    peak_overshoot = 0
    iterations = 0
    last_angle = start_angle
    last_change = 0
    reached = False

    log("TURN", "begin start_angle={} target={} delta={}".format(
        start_angle, target, target - start_angle))

    while t.time() < timeout:
        angle = heading()
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
            settled = heading()
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
    final = heading()
    if reached:
        log("TURN", "DONE target={} final={} peak_overshoot={} iterations={} time={}ms".format(
            target, final, peak_overshoot, iterations, t.time()))
    else:
        log("TURN", "TIMEOUT target={} final={} peak_overshoot={} iterations={} time={}ms".format(
            target, final, peak_overshoot, iterations, t.time()))

    # Readings from before or during the turn say nothing about where the robot
    # is now, so throw them away and let the vote start fresh.
    in_turn = False
    reset_color_debounce()


def start_turn(delta, then):
    global turn_target
    before = heading()
    turn_target = round((before + delta) / 90) * 90
    log("TURN", "start_turn delta={} gyro_now={} -> turn_target={} then={}".format(
        delta, before, turn_target, then))
    turn_to(turn_target)
    set_state(then, "turn complete")


def autopilot_start():
    global autopilot_active, turn_target, heading_offset
    log("AUTO", "START - resetting gyro (was {})".format(heading()))
    gyro.reset_angle(0)
    heading_offset = 0
    turn_target = 0
    autopilot_active = True
    set_state("STRAIGHT", "autopilot start")


def autopilot_stop():
    global autopilot_active
    log("AUTO", "STOP requested")
    autopilot_active = False
    stop_all()


# ---------------------------------------------------------------- color detection
def beep(times):
    for _ in range(times):
        ev3.speaker.beep(frequency=500, duration=100)
        wait(200)


def do_pause():
    """Stop for PAUSE_MS, then carry on. Autopilot re-issues its drive command on the next loop."""
    global state_start
    log("COLOR", "PAUSE for {}ms".format(PAUSE_MS))
    t0 = clock.time()
    stop_all()
    wait(PAUSE_MS)
    # Push the state timer forward so timed states (BACKUP, CLEAR_CORNER, ...)
    # don't count the pause as driving time.
    state_start += clock.time() - t0


def do_manual():
    """Hand control to the driver. Send 'p' to start the autopilot again."""
    global autopilot_active
    if autopilot_active:
        autopilot_active = False
        stop_all()
        log("COLOR", "MANUAL: autopilot off, driver in control (send 'p' to resume)")
    else:
        log("COLOR", "MANUAL: already under manual control")
    beep(2)


def do_finish():
    """End of the maze: stop and turn autopilot off. Ignored until armed with 'o'."""
    global autopilot_active, finish_armed
    if not finish_armed:
        log("COLOR", "FINISH color seen but not armed (press 'o' to arm), ignoring")
        return
    finish_armed = False        # one-shot: press 'o' again to re-arm
    if autopilot_active:
        autopilot_active = False
        stop_all()
        log("COLOR", "FINISH: end of maze, autopilot off")
    else:
        log("COLOR", "FINISH: color seen under manual control (not stopping the driver)")
    beep(3)


COLOR_HANDLERS = {
    "PAUSE": do_pause,
    "MANUAL": do_manual,
    "FINISH": do_finish,
}


def reset_color_debounce():
    """Forget the color being confirmed (called after every turn)."""
    global candidate, confidence
    candidate = None
    confidence = 0


def check_color():
    """Debounced, edge-triggered color check: an action fires once when a new color is confirmed."""
    global last_color, candidate, confidence
    if in_turn:
        return

    color = color_sensor.color()

    if color == candidate:
        confidence = min(confidence + 1, STABLE_READINGS)
    else:
        candidate = color
        confidence = 1

    if confidence >= STABLE_READINGS and candidate != last_color:
        last_color = candidate
        action = None
        for c, a in COLOR_ACTIONS:
            if c == candidate:
                action = a
        log("COLOR", "confirmed {} -> action {}".format(candidate, action))
        if action is not None:
            handler = COLOR_HANDLERS.get(action)
            if handler is None:
                log("COLOR", "WARN unknown action '{}' in COLOR_ACTIONS".format(action))
            else:
                handler()


# ---------------------------------------------------------------- autopilot
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
        try_snap(dist)
        wall_error = dist - TARGET_DIST
        offset = max(-WALL_ANGLE_MAX, min(WALL_ANGLE_MAX, WALL_ANGLE_GAIN * wall_error))
        if log_due():
            log("FOLLOW", "dist={} wall_err={} offset={} gyro={}".format(
                dist, wall_error, offset, heading()))
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
        dist = ultra_sensor.distance()
        if bumper.pressed():
            set_state("BACKUP", "bumper pressed in ENTER")
        elif elapsed() > 1000 and dist < OPEN_DIST:
            set_state("FOLLOW", "right wall found, dist={}".format(dist))
        elif elapsed() > ENTER_MAX_MS:
            set_state("FOLLOW", "ENTER timed out, no wall (dist={})".format(dist))


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
                elif key == "o":
                    finish_armed = True
                    log("AUTO", "FINISH armed: the next FINISH color will end the run")
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

        # Color check runs every loop, in both manual and autopilot mode.
        # If it turns autopilot off (MANUAL / FINISH), the step below is skipped.
        check_color()

        if autopilot_active:
            autopilot_step()

except Exception as e:
    log("ERROR", repr(e))
    raise
finally:
    log("EXIT", "shutting down")
    stop_all()
    client_socket.close()
    server_socket.close()
