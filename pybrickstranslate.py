from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
import runloop

# Port mapping
LEFT_PORT = Port.F
RIGHT_PORT = Port.B
FRONT_PORT = Port.E
BACK_PORT = Port.A
SENSOR_PORT = Port.C


class Sensor:
    """Sensor wrapper for reading angles and strengths."""

    def __init__(self, sensor_port=SENSOR_PORT):
        self.port = sensor_port
        self.color_sensor = ColorSensor(sensor_port)

    def front_angle(self):
        return self.color_sensor.reflection()

    def front_strength(self):
        return self.color_sensor.rgb()[0]

    def back_angle(self):
        return self.color_sensor.rgb()[2]

    def back_strength(self):
        return self.color_sensor.rgb()[1]

    def read(self):
        r = (
            self.front_angle(),
            self.front_strength(),
            self.back_angle(),
            self.back_strength(),
        )
        print("SENSOR READ:\n{}".format(r))
        return r


class MotorController:
    """Encapsulates motor port operations and movement primitives."""

    def __init__(self, left=LEFT_PORT, right=RIGHT_PORT, front=FRONT_PORT, back=BACK_PORT):
        self.left = Motor(left)
        self.right = Motor(right)
        self.front = Motor(front)
        self.back = Motor(back)

    def run_motor(self, motor, speed):
        print("MOTOR RUN:\nmotor={} speed={}".format(motor, speed))
        motor.run(speed)

    def run_left(self, speed):
        self.run_motor(self.left, speed)

    def run_right(self, speed):
        self.run_motor(self.right, speed)

    def run_front(self, speed):
        self.run_motor(self.front, speed)

    def run_back(self, speed):
        self.run_motor(self.back, speed)

    def stop_all(self):
        self.run_back(0)
        self.run_front(0)
        self.run_left(0)
        self.run_right(0)

    # Movement helpers
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

    def go_north_west(self, speed):
        self.run_front(speed)
        self.run_back(-speed)
        self.run_left(-speed)
        self.run_right(speed)

    def go_north_east(self, speed):
        self.run_front(-speed)
        self.run_back(speed)
        self.run_left(speed)
        self.run_right(-speed)

    def go_south_west(self, speed):
        self.run_front(speed)
        self.run_back(-speed)
        self.run_left(speed)
        self.run_right(-speed)

    def go_south_east(self, speed):
        self.run_front(-speed)
        self.run_back(speed)
        self.run_left(-speed)
        self.run_right(speed)

    def rotate(self, speed):
        self.run_left(speed)
        self.run_right(speed)
        self.run_back(speed)
        self.run_front(speed)


class Robot:
    """High-level robot behavior combining sensors and motors."""

    def __init__(self, sensor=None, motors=None):
        self.sensor = sensor or Sensor()
        self.motors = motors or MotorController()

    # Direction predicates
    @staticmethod
    def ball_front(direction):
        return direction == 20

    @staticmethod
    def ball_behind(direction):
        return direction == 5

    @staticmethod
    def ball_dead_left(direction):
        return direction == 1

    @staticmethod
    def ball_dead_right(direction):
        return direction == 9

    @staticmethod
    def is_north_west(direction):
        return direction in {2, 3, 4}

    @staticmethod
    def is_north_east(direction):
        return direction in {6, 7, 8}

    @staticmethod
    def is_south_west(direction):
        return direction in {6, 7, 8}

    @staticmethod
    def is_south_east(direction):
        return direction in {2, 3, 4}

    @staticmethod
    def ball_undetected(direction):
        return direction == 0

    def should_robot_stop(self, front_angle_val, back_angle_val):
        return self.ball_front(front_angle_val) or self.ball_behind(back_angle_val)

    def face_ball(self, speed):
        while True:
            f_angle, f_strength, b_angle, b_strength = self.sensor.read()
            print((f_angle, f_strength, b_angle, b_strength))
            if f_angle != 20:
                self.motors.rotate(speed)
            else:
                self.motors.stop_all()
                print("Ball is in the front")
                break

    def kick_ball_and_stop(self, speed):
        self.motors.go_forward(speed)
        while True:
            f_angle, f_strength, b_angle, b_strength = self.sensor.read()
            if self.should_robot_stop(f_angle, b_angle):
                self.motors.stop_all()
                break

    async def main(self):
        k = 80
        vy = 1000
        base_v = 700
        max_v = 1000
        while True:
            f_angle, f_strength, b_angle, b_strength = self.sensor.read()
            delta_front = f_angle - 20
            delta_back = b_angle - 20
            print(f_angle, b_angle)
            if f_angle != 0 and b_angle in {0, 4, 8, 12, 16, 20, 24, 28, 32, 36}: #Probably add extra conditions here for example Distance_Sensor <= x cm
                print("front sensor activated")
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
                    self.motors.go_right(vx)
            elif b_angle != 0 and f_angle == 0: #Here as well Distance_Sensor <= x cm
                print("back sensor activated")
                vx = delta_back * k
                print("vx = ", vx)
                if b_angle in {20}:
                    self.motors.run_right(0)
                    self.motors.run_left(0)
                    self.motors.go_left(vy)
                elif b_angle in {4, 36}:
                    self.motors.run_front(0)
                    self.motors.run_back(0)
                    self.motors.go_backwards(vy)
                else: #instead use a elif statement here instead of else
                    self.motors.run_front(0)
                    self.motors.run_back(0)
                    self.motors.go_backwards(vy)

                    # An else statement should be used here
                    # So if none of the conditions above were filled(aka robot went past the midpoint line) start moving back
                    # E.g motor_run backwards

def main(
    left_port=LEFT_PORT,
    right_port=RIGHT_PORT,
    front_port=FRONT_PORT,
    back_port=BACK_PORT,
    sensor_port=SENSOR_PORT,
):
    """Initialize robot components with given ports and start main loop."""
    sensor = Sensor(sensor_port)
    motors = MotorController(left=left_port, right=right_port, front=front_port, back=back_port)
    robot = Robot(sensor=sensor, motors=motors)
    runloop.run(robot.main())


if __name__ == "__main__":
    main()