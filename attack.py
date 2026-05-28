"""
Pybricks port of the original `attack.py` MovementRobot.

This keeps the original behavior and logging while using Pybricks
`Motor` and `ColorSensor`. The color sensor readings are mapped into
0-36-like "angle" buckets to preserve the original decision logic.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait


# Custom logger
_log_counter = 0


def log(level, message):
    global _log_counter
    _log_counter += 1
    print("[{}] : {}".format(level, message))


def _map_to_angle_bucket(value, in_min=0, in_max=100, out_max=36):
    """Map sensor value (e.g. reflection 0-100) to 0..out_max bucket."""
    try:
        v = int((value - in_min) * out_max / (in_max - in_min) + 0.5)
    except Exception:
        return 0
    return max(0, min(out_max, v))


class MovementRobot:
    LEFT_MOTOR = Port.F
    RIGHT_MOTOR = Port.B
    FRONT_MOTOR = Port.E
    BACK_MOTOR = Port.A
    SENSOR_PORT = Port.C

    VALID_FRONT_SPREAD = {0, 4, 8, 12, 16, 20, 24, 28, 32, 36}

    def __init__(self):
        self.hub = PrimeHub()
        self.left = Motor(self.LEFT_MOTOR)
        self.right = Motor(self.RIGHT_MOTOR)
        self.front = Motor(self.FRONT_MOTOR)
        self.back = Motor(self.BACK_MOTOR)
        self.color = ColorSensor(self.SENSOR_PORT)

        self.turn_gain = 80
        self.drive_speed = 1000
        log("INFO", "MovementRobot initialized with turn_gain=80, drive_speed=1000")

    def front_sensor_angle(self):
        # map reflection (0-100) into 0-36 pseudo-angle
        return _map_to_angle_bucket(self.color.reflection())

    def back_sensor_angle(self):
        r, g, b = self.color.rgb()
        # use blue channel as a proxy and map 0-255 into 0-36
        return _map_to_angle_bucket(b, in_min=0, in_max=255)

    def front_sensor_strength(self):
        r, g, b = self.color.rgb()
        return r

    def back_sensor_strength(self):
        r, g, b = self.color.rgb()
        return g

    def read_sensor(self):
        result = (
            self.front_sensor_angle(),
            self.front_sensor_strength(),
            self.back_sensor_angle(),
            self.back_sensor_strength(),
        )
        log("DEBUG", "Sensor reading: angle={}, front_strength={}, back_angle={}, back_strength={}".format(result[0], result[1], result[2], result[3]))
        return result

    def run_left_motor(self, speed):
        self.left.run(int(speed))

    def run_right_motor(self, speed):
        self.right.run(int(speed))

    def run_front_motor(self, speed):
        self.front.run(int(speed))

    def run_back_motor(self, speed):
        self.back.run(int(speed))

    def go_forward(self, speed):
        log("INFO", "Robot moving forward with speed={}".format(speed))
        self.run_left_motor(-speed)
        self.run_right_motor(speed)

    def go_backwards(self, speed):
        log("INFO", "Robot moving backwards with speed={}".format(speed))
        self.run_left_motor(speed)
        self.run_right_motor(-speed)

    def go_left(self, speed):
        log("INFO", "Robot moving left with speed={}".format(speed))
        self.run_front_motor(speed)
        self.run_back_motor(-speed)

    def go_right(self, speed):
        log("INFO", "Robot moving right with speed={}".format(speed))
        self.run_front_motor(-speed)
        self.run_back_motor(speed)

    def go_north_west(self, speed):
        self.run_front_motor(speed)
        self.run_back_motor(-speed)
        self.run_left_motor(-speed)
        self.run_right_motor(speed)

    def go_north_east(self, speed):
        self.run_front_motor(-speed)
        self.run_back_motor(speed)
        self.run_left_motor(speed)
        self.run_right_motor(-speed)

    def go_south_west(self, speed):
        self.run_front_motor(speed)
        self.run_back_motor(-speed)
        self.run_left_motor(speed)
        self.run_right_motor(-speed)

    def go_south_east(self, speed):
        self.run_front_motor(-speed)
        self.run_back_motor(speed)
        self.run_left_motor(-speed)
        self.run_right_motor(speed)

    def robot_rotate(self, speed):
        log("INFO", "Robot rotating with speed={}".format(speed))
        self.run_left_motor(speed)
        self.run_right_motor(speed)
        self.run_back_motor(speed)
        self.run_front_motor(speed)

    @staticmethod
    def ball_front(direction):
        return direction in range(16, 25)

    @staticmethod
    def ball_behind(direction):
        return direction in range(0, 9)

    @staticmethod
    def ball_dead_left(direction):
        return direction == 1

    @staticmethod
    def ball_dead_right(direction):
        return direction == 9

    @staticmethod
    def north_west(direction):
        return direction in {2, 3, 4}

    @staticmethod
    def north_east(direction):
        return direction in {6, 7, 8}

    @staticmethod
    def south_west(direction):
        return direction in {6, 7, 8}

    @staticmethod
    def south_east(direction):
        return direction in {2, 3, 4}

    @staticmethod
    def ball_undetected(direction):
        return direction == 0

    def stop_all_motors(self):
        self.run_back_motor(0)
        self.run_front_motor(0)
        self.run_left_motor(0)
        self.run_right_motor(0)

    def face_ball(self, speed):
        while True:
            signal = self.read_sensor()
            print(signal)
            if not self.ball_front(signal[0]):
                self.robot_rotate(speed)
            else:
                self.stop_all_motors()
                print("Ball is in the front")
                break

    def should_robot_stop(self, front_sensor_angle, back_sensor_angle):
        return self.ball_front(front_sensor_angle) or self.ball_behind(back_sensor_angle)

    def kick_ball_and_stop(self, speed):
        self.go_forward(speed)
        while True:
            signal = self.read_sensor()
            if self.should_robot_stop(signal[0], signal[2]):
                self.stop_all_motors()
                break

    def main(self):
        while True:
            front_sensor_angle, front_sensor_strength, back_sensor_angle, back_sensor_strength = self.read_sensor()
            delta_front = front_sensor_angle - 20
            delta_back = back_sensor_angle - 20

            print(front_sensor_angle, back_sensor_angle)

            if front_sensor_angle != 0 and back_sensor_angle in self.VALID_FRONT_SPREAD:
                print("front sensor activated")
                vx = delta_front * self.turn_gain
                if front_sensor_angle in {12, 16, 20, 24, 28}:
                    self.run_front_motor(0)
                    self.run_back_motor(0)
                    self.go_forward(self.drive_speed)
                elif front_sensor_angle in {4, 32}:
                    self.run_front_motor(0)
                    self.run_back_motor(0)
                    self.go_backwards(self.drive_speed)
                else:
                    self.go_forward(self.drive_speed)
                    self.go_right(vx)
            elif back_sensor_angle != 0 and front_sensor_angle == 0:
                print("back sensor activated")
                vx = delta_back * self.turn_gain
                print("vx =", vx)
                if back_sensor_angle in {16, 20, 24}:
                    self.run_right_motor(0)
                    self.run_left_motor(0)
                    self.go_left(self.drive_speed)
                elif back_sensor_angle in {0, 4, 8, 32, 36}:
                    self.run_front_motor(0)
                    self.run_back_motor(0)
                    self.go_backwards(self.drive_speed)
                else:
                    self.run_front_motor(0)
                    self.run_back_motor(0)
                    self.go_backwards(self.drive_speed)


def run():
    robot = MovementRobot()
    try:
        while True:
            robot.main()
            wait(10)
    except KeyboardInterrupt:
        robot.stop_all_motors()


if __name__ == "__main__":
    run()