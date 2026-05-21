import motor
from hub import port
import runloop
import color_sensor

# Custom logger without logging module
_log_counter = 0

def log(level, message):
    global _log_counter
    _log_counter += 1
    print("[{}] : {}".format(level, message))


class MovementRobot:
    LEFT_MOTOR = port.F
    RIGHT_MOTOR = port.B
    FRONT_MOTOR = port.E
    BACK_MOTOR = port.A
    SENSOR_PORT = port.C

    VALID_FRONT_SPREAD = {0, 4, 8, 12, 16, 20, 24, 28, 32, 36}

    def __init__(self):
        self.turn_gain = 80
        self.drive_speed = 1000
        log("INFO", "MovementRobot initialized with turn_gain=80, drive_speed=1000")

    def front_sensor_angle(self):
        return color_sensor.reflection(self.SENSOR_PORT)

    def back_sensor_angle(self):
        color = color_sensor.rgbi(self.SENSOR_PORT)
        return color[2]

    def front_sensor_strength(self):
        color = color_sensor.rgbi(self.SENSOR_PORT)
        return color[0]

    def back_sensor_strength(self):
        color = color_sensor.rgbi(self.SENSOR_PORT)
        return color[1]

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
        motor.run(self.LEFT_MOTOR, speed)

    def run_right_motor(self, speed):
        motor.run(self.RIGHT_MOTOR, speed)

    def run_front_motor(self, speed):
        motor.run(self.FRONT_MOTOR, speed)

    def run_back_motor(self, speed):
        motor.run(self.BACK_MOTOR, speed)

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

    async def main(self):
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


runloop.run(MovementRobot().main())