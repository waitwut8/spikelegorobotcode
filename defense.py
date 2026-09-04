import runloop, motor, color_sensor, distance_sensor, time
from hub import port, motion_sensor

LP, RP, FP, BP, SP, WP = port.E, port.A, port.F, port.B, port.C, port.D

HALFWAY_MARK_MM = 150# fill in real value: ultrasonic distance that triggers the reverse
QUARTER_MARK_MM = 75    # fill in real value: distance the reverse must clear past before re-approaching
REVERSE_SPEED = 400    # tunable reverse speed

KP = .8
DEAD_ENTER = 15    # yaw magnitude (deg) at which correction turns ON
DEAD_EXIT = 8        # yaw magnitude (deg) at which correction turns back OFF (hysteresis)
MAX = 120
LIMIT = 120
TIMEOUT = 900
RECOVERED = 60
RECOVERY = 1200
YAW_FILTER_ALPHA = 0.3# 0 = ignore new readings, 1 = no smoothing at all
MAX_STEP = 20            # largest change in correction allowed per cycle (slew-rate limit)

RECOVERY_CHECK_MIN = 400        # cycles between recovery checks when yaw is way past LIMIT (checked often)
RECOVERY_CHECK_MAX = 2000    # cycles between recovery checks when yaw is only just past LIMIT (checked rarely)
RECOVERY_CHECK_SCALE_YAW = 180# yaw magnitude (deg) at which the interval bottoms out at RECOVERY_CHECK_MIN

_yaw_offcourse_start = None
_yaw_filtered = None
_correcting = False
_last_correction = 0
_last_recovery_check = 0


def log(message, *values):
    print(message.format(*values))


def get_yaw():
    return motion_sensor.tilt_angles()[0]


def filtered_yaw():
    """Exponential low-pass filter on the raw yaw reading, to smooth out sensor noise."""
    global _yaw_filtered
    raw = get_yaw()
    _yaw_filtered = raw if _yaw_filtered is None else _yaw_filtered + YAW_FILTER_ALPHA * (raw - _yaw_filtered)
    return _yaw_filtered


def turn_sign_toward_zero_yaw(yaw):
    return 0 if not yaw else (1 if KP >= 0 else -1) * (1 if yaw > 0 else -1)


def yaw_correction(max_correction=MAX):
    """P-controller with hysteresis (to stop chatter at the deadband edge) and
    a slew-rate limit (to stop the output jumping between cycles)."""
    global _correcting, _last_correction

    yaw = filtered_yaw()
    mag = abs(yaw)

    if _correcting:
        if mag < DEAD_EXIT:
            _correcting = False
    else:
        if mag > DEAD_ENTER:
            _correcting = True

    target = 0 if not _correcting else turn_sign_toward_zero_yaw(yaw) * min(max_correction, int(abs(KP) * mag))

    delta = target - _last_correction
    if delta > MAX_STEP:
        delta = MAX_STEP
    elif delta < -MAX_STEP:
        delta = -MAX_STEP
    _last_correction += delta

    return _last_correction


def recovery_check_interval():
    """Cycles to wait between recovery checks, scaled by how far off heading we
    currently are: near LIMIT -> checked rarely, way past LIMIT -> checked often."""
    mag = abs(get_yaw())
    fraction = min(1, mag / RECOVERY_CHECK_SCALE_YAW)
    return int(RECOVERY_CHECK_MAX - fraction * (RECOVERY_CHECK_MAX - RECOVERY_CHECK_MIN))


def recovery_check_due(cycles):
    global _last_recovery_check
    if cycles - _last_recovery_check >= recovery_check_interval():
        _last_recovery_check = cycles
        return True
    return False


def yaw_recovery_needed():
    global _yaw_offcourse_start

    yaw = get_yaw()

    if abs(yaw) > LIMIT:
        if _yaw_offcourse_start is None:
            _yaw_offcourse_start = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), _yaw_offcourse_start) > TIMEOUT:
            return True
    else:
        _yaw_offcourse_start = None

    return False


async def recover_heading(robot):
    global _yaw_offcourse_start

    recovery_start = time.ticks_ms()
    while abs(get_yaw()) > RECOVERED and time.ticks_diff(time.ticks_ms(), recovery_start) <= RECOVERY:
        yaw = get_yaw()
        speed = turn_sign_toward_zero_yaw(yaw) * 700
        robot.m.apply((speed,) * 4)
        log("recovering yaw={} speed={}", yaw, speed)
        await runloop.sleep_ms(10)

    robot._go("stop")
    _yaw_offcourse_start = None


class Sensor:
    def __init__(self, port_=SP):
        self.port = port_

    def read(self):
        reflection = color_sensor.reflection(self.port)
        r, g, b, _ = color_sensor.rgbi(self.port)
        data = reflection // 4, r, b // 4, g
        return data


class SimulatedSensor:
    
    _SWEEP = [1] * 5 + [3] * 5 + [5] * 5 + [7] * 5 + [9] * 5 + [7] * 5 + [5] * 5 + [3] * 5

    PATTERNS = {
        "strafe": [3],                # constant front reading
        "strafe_sweep": _SWEEP,    # front sweeps low -> high -> low
        "back_strafe": [3],        # constant back reading
        "back_sweep": _SWEEP,        # back sweeps low -> high -> low
        "idle": [0],                # nothing detected
        "approach": list(range(9, 4, -1)),# front value ramps down toward the stop threshold (5)
    }

    def __init__(self, mode="strafe"):
        self.mode = mode
        self.step = 0
        self.pattern = self.PATTERNS.get(mode, [0])

    def read(self):
        value = self.pattern[self.step % len(self.pattern)]
        self.step += 1

        if self.mode.startswith("back"):
            f, b = 0, value
        else:
            f, b = value, 0

        data = (f, 0, b, 0)
        return data


class MotorController:
    def __init__(self):
        self.ports = LP, RP, FP, BP
        self.last_speeds = (0, 0, 0, 0)

    def apply(self, speeds):
        self.last_speeds = speeds
        for p, spd in zip(self.ports, speeds):
            motor.run(p, spd)

    def stop(self):
        self.last_speeds = (0, 0, 0, 0)
        for p in self.ports:
            motor.stop(p)


class Robot:
    MOVE = {
        "fwd": lambda s: (-s, s, 0, 0),
        "back": lambda s: (s, -s, 0, 0),
        "left": lambda s: (0, 0, s, -s),
        "right": lambda s: (0, 0, -s, s),
        "nw": lambda s: (-s, s, s, -s),
        "ne": lambda s: (s, -s, -s, s),
        "sw": lambda s: (s, -s, s, -s),
        "se": lambda s: (-s, s, -s, s),
        "rot": lambda s: (s, s, s, s),
        "stop": lambda s: (0, 0, 0, 0),
    }

    def __init__(self):
        self.s = SimulatedSensor("strafe_sweep")
        self.m = MotorController()
        self.k = 80
        self.vy = 1000
        self.state = "idle"
        self.cycles = 0
        self.front_angles = ((0, 1), (1, -1), (1, -1), (1, 1), (0, 1), (0, 1), (0, 1), (1, 1), (1, -1), (1, 1))
        self.back_angles = ((0, -1),) * 4 + ((1, 0),) * 2 + ((0, -1),) * 4

    def _go(self, name, speed=0, rotation=0):
        self.m.apply(tuple(x + rotation for x in self.MOVE[name](speed)))

    def _vector(self, x, y, rotation=0):
        self.m.apply((-y + rotation, y + rotation, x + rotation, -x + rotation))

    def should_robot_stop(self, f, b):
        return f == 5 or b == 1

    async def face_ball(self, speed):
        while True:
            f, _, b, _ = self.s.read()

            if f != 20:
                self._go("rot", speed)
            else:
                self._go("stop")
                log("Ball is in the front")
                break

            await runloop.sleep_ms(10)

    async def kick_ball_and_stop(self, speed):
        self._go("fwd", speed)

        while True:
            f, _, b, _ = self.s.read()
            dist = distance_sensor.distance(WP)

            if dist != -1 and dist <= HALFWAY_MARK_MM:
                await self._reverse_past_quarter()
                self._go("fwd", speed)

            if self.should_robot_stop(f, b):
                self._go("stop")
                break

            await runloop.sleep_ms(10)

    async def _reverse_past_quarter(self):
        self._go("back", REVERSE_SPEED)

        while True:
            dist = distance_sensor.distance(WP)

            if dist != -1 and dist >= QUARTER_MARK_MM:
                self._go("stop")
                break

            await runloop.sleep_ms(10)

    def _front(self, f, rotation=0):
        f = max(0, min(f, len(self.front_angles) - 1))
        vx = (f - 5) * self.k
        x, y = self.front_angles[f]
        self._vector(x * vx if x else 0, y * self.vy, rotation)

    def _back(self, b, rotation=0):
        b = max(0, min(b, len(self.back_angles) - 1))
        vx = (b - 5) * self.k
        x, y = self.back_angles[b]
        self._vector(x * self.vy if x else 0, y * self.vy if y else 0, rotation)

    async def main(self):
        handlers = {
            "front": self._front,
            "back": self._back
        }

        while True:
            self.cycles += 1

            f, _, b, _ = self.s.read()

            if f:
                self.state = "front"
            elif b:
                self.state = "back"
            else:
                self.state = "idle"

            rotation = yaw_correction()
            if recovery_check_due(self.cycles) and yaw_recovery_needed():
                await recover_heading(self)
                continue

            handler = handlers.get(self.state)

            if handler:
                handler(f if self.state == "front" else b, rotation)
            else:
                self._go("stop", rotation=rotation)

            log("cycle={} state={} f={} b={} rotation={} motors={}",
                self.cycles, self.state, f, b, rotation, self.m.last_speeds)

            await runloop.sleep_ms(10)


runloop.run(Robot().main())