"""
Pybricks port of original `otherthing.py`.

Features preserved:
- Sensor wrapper compatible with original reads
- MotorController with inversion, scaling and simulate helpers
- Robot main loop using front/back sensor logic

Notes:
- Interactive CLI removed (not available on hub). Use constants below
  to tune `inversions` and `scale` before deploying.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait


# Port mapping (adjust to your wiring)
LEFT_PORT = Port.F
RIGHT_PORT = Port.B
FRONT_PORT = Port.E
BACK_PORT = Port.A
SENSOR_PORT = Port.C


class Sensor:
    def __init__(self, port=SENSOR_PORT):
        self.color = ColorSensor(port)

    def front_angle(self):
        # use reflection as a rough angle/position cue
        return self.color.reflection()

    def front_strength(self):
        r, g, b = self.color.rgb()
        return r

    def back_angle(self):
        r, g, b = self.color.rgb()
        return b

    def back_strength(self):
        r, g, b = self.color.rgb()
        return g

    def read(self):
        return (self.front_angle(), self.front_strength(), self.back_angle(), self.back_strength())


class MotorController:
    """Wrap pybricks Motor objects with inversion, scale and helpers."""

    def __init__(self, left: Motor, right: Motor, front: Motor, back: Motor, max_speed=1000, inversions=None, scale=1.0):
        self.left = left
        self.right = right
        self.front = front
        self.back = back
        self.max_speed = max_speed
        self.inversions = inversions or {"left": 1, "right": 1, "front": 1, "back": 1}
        self.scale = float(scale)
        self._last = {"left": None, "right": None, "front": None, "back": None}

    def _apply(self, key: str, motor: Motor, speed: float, force: bool = False):
        inv = self.inversions.get(key, 1)
        scaled = int(max(-self.max_speed, min(self.max_speed, speed * self.scale * inv)))
        if force or self._last.get(key) != scaled:
            motor.run(scaled)
            self._last[key] = scaled

    def run_left(self, speed, force=False): self._apply("left", self.left, speed, force)
    def run_right(self, speed, force=False): self._apply("right", self.right, speed, force)
    def run_front(self, speed, force=False): self._apply("front", self.front, speed, force)
    def run_back(self, speed, force=False): self._apply("back", self.back, speed, force)

    def stop_all(self):
        self.run_left(0, force=True)
        self.run_right(0, force=True)
        self.run_front(0, force=True)
        self.run_back(0, force=True)

    def dump_config(self):
        return {"inversions": self.inversions.copy(), "scale": self.scale, "max_speed": self.max_speed}

    def set_scale(self, v):
        self.scale = float(v)

    def toggle_inversion(self, key):
        self.inversions[key] = -1 * self.inversions.get(key, 1)

    def simulate_speed(self, key, speed):
        inv = self.inversions.get(key, 1)
        return int(max(-self.max_speed, min(self.max_speed, speed * self.scale * inv)))

    # Movement helpers (differential drive / pseudo-holonomic mapping preserved)
    def go_forward(self, speed):
        self.run_left(-speed)
        self.run_right(speed)

    def go_backwards(self, speed):
        self.run_left(speed)
        self.run_right(-speed)

    def go_left(self, speed):
        self.run_front(speed)
        self.run_back(-speed)

    def go_right(self, speed):
        self.run_front(-speed)
        self.run_back(speed)

    def rotate(self, speed):
        self.run_left(speed)
        self.run_right(speed)
        self.run_front(speed)
        self.run_back(speed)


class Robot:
    def __init__(self, sensor: Sensor, motors: MotorController):
        self.sensor = sensor
        self.motors = motors

    @staticmethod
    def ball_front(direction):
        return direction == 20

    @staticmethod
    def ball_behind(direction):
        return direction == 5

    def should_robot_stop(self, front_angle_val, back_angle_val):
        return self.ball_front(front_angle_val) or self.ball_behind(back_angle_val)

    def face_ball(self, speed):
        while True:
            f_angle, f_strength, b_angle, b_strength = self.sensor.read()
            if f_angle != 20:
                self.motors.rotate(speed)
            else:
                self.motors.stop_all()
                break

    def kick_ball_and_stop(self, speed):
        self.motors.go_forward(speed)
        while True:
            f_angle, f_strength, b_angle, b_strength = self.sensor.read()
            if self.should_robot_stop(f_angle, b_angle):
                self.motors.stop_all()
                break

    def main(self):
        k = 80
        vy = 700
        while True:
            f_angle, f_strength, b_angle, b_strength = self.sensor.read()
            delta_front = f_angle - 20
            delta_back = b_angle - 20

            front_sees = f_angle != 0
            back_sees = b_angle != 0

            if front_sees and (not back_sees):
                vx = delta_front * k
                if f_angle in {16, 20, 24}:
                    self.motors.run_front(0)
                    self.motors.run_back(0)
                    self.motors.go_forward(vy)
                elif f_angle in {4, 32}:
                    self.motors.run_front(0)
                    self.motors.run_back(0)
                    self.motors.go_backwards(vy)
                else:
                    self.motors.go_forward(vy)
                    # simple lateral correction via front/back motors
                    self.motors.go_right(vx)

            elif back_sees and (not front_sees):
                vx = delta_back * k
                if b_angle == 20:
                    self.motors.run_right(0)
                    self.motors.run_left(0)
                    self.motors.go_left(vy)
                elif b_angle in {4, 36}:
                    self.motors.run_front(0)
                    self.motors.run_back(0)
                    self.motors.go_backwards(vy)
                else:
                    self.motors.run_front(0)
                    self.motors.run_back(0)
                    self.motors.go_backwards(vy)

            else:
                self.motors.stop_all()


def main():
    hub = PrimeHub()

    # Create motor objects
    left_m = Motor(LEFT_PORT)
    right_m = Motor(RIGHT_PORT)
    front_m = Motor(FRONT_PORT)
    back_m = Motor(BACK_PORT)

    # Default calibration: set inversions and scale here
    inversions = {"left": -1, "right": 1, "front": 1, "back": 1}
    motors = MotorController(left_m, right_m, front_m, back_m, max_speed=1000, inversions=inversions, scale=0.95)
    sensor = Sensor(SENSOR_PORT)
    robot = Robot(sensor, motors)

    try:
        while True:
            robot.main()
            wait(10)
    except KeyboardInterrupt:
        motors.stop_all()


if __name__ == "__main__":
    main()