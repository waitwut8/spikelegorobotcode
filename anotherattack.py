import motor
from hub import port
import runloop
import color_sensor
import time
import distance_sensor

################# constants here  #################
k_LEFT_PORT = port.F
k_RIGHT_PORT = port.B
k_FRONT_PORT = port.E
k_BACK_PORT = port.A
k_SENSOR_PORT = port.C
k_CENTER_ANGLE = 20
k_BEHIND_ANGLE = 5
k_DEAD_LEFT = 1
k_DEAD_RIGHT = 9
k_MAX_SPEED = 1000

################# constants done  #################


class Sensor:
    """
    sensor wrapper for reading angles and strengths
    qikeasy ir uses some wacky stuff involving RGB so like uhh yeah
    """
    def __init__(self, sensor_port=k_SENSOR_PORT):
        self.m_port = sensor_port
    def m_front_angle(self):
        return color_sensor.reflection(self.m_port)
    def m_front_strength(self):
        return color_sensor.rgbi(self.m_port)[0]
    def m_back_angle(self):
        return color_sensor.rgbi(self.m_port)[2]
    def m_back_strength(self):
        return color_sensor.rgbi(self.m_port)[1]
    def m_read(self):
        r = (
            self.m_front_angle(),
            self.m_front_strength(),
            self.m_back_angle(),
            self.m_back_strength(),
        )
        print("SENSOR READ:\n{}".format(r))
        return r


class MotorController:
    """
    motor port operations and movement primitives
    uses lambda operations to flex using mathematics
    why do i even study specialist anymore
    """
    def __init__(
        self,
        left=k_LEFT_PORT,
        right=k_RIGHT_PORT,
        front=k_FRONT_PORT,
        back=k_BACK_PORT,
 ):
        self.m_left = left
        self.m_right = right
        self.m_front = front
        self.m_back = back
        #aneurysms zone
        self.m_left_speed = lambda x, y, r: -y + r
        self.m_right_speed = lambda x, y, r: y + r
        self.m_front_speed = lambda x, y, r: x + r
        self.m_back_speed = lambda x, y, r: -x + r
        #aneuryms zone done
    
    @staticmethod
    def m_clamp(speed):
        return max(-k_MAX_SPEED, min(k_MAX_SPEED, speed))
    
    def m_run_motor(self, motor_port, speed):
        speed = self.m_clamp(speed)
        print("MOTOR RUN:\nport={} speed={}".format(motor_port, speed))
        motor.run(motor_port, speed)

    def m_drive_vector(self, x, y, r=0):
        # essentially 2d vectoring with x, y in cartesian and r in rotation
        self.m_run_motor(self.m_left, self.m_left_speed(x, y, r))
        self.m_run_motor(self.m_right, self.m_right_speed(x, y, r))
        self.m_run_motor(self.m_front, self.m_front_speed(x, y, r))
        self.m_run_motor(self.m_back, self.m_back_speed(x, y, r))

    def m_stop_all(self):
        self.m_drive_vector(0, 0, 0)
class Robot:
    """
    i pray to god this works
    """
    def __init__(self, sensor=None, motors=None):
        self.m_sensor = sensor or Sensor()
        self.m_motors = motors or MotorController()
    #the sheer amount of staticmethod annontations are from my habits mb guys

    @staticmethod
    def m_ball_front(direction):
        return direction == 20
    
    @staticmethod
    def m_ball_behind(direction):
        return direction == 5
    
    @staticmethod
    def m_ball_dead_left(direction):
        return direction == 1
    
    @staticmethod
    def m_ball_dead_right(direction):
        return direction == 9
    
    @staticmethod
    def m_is_north_west(direction):
        return direction in {2, 3, 4}
    
    @staticmethod
    def m_is_north_east(direction):
        return direction in {6, 7, 8}
    
    @staticmethod
    def m_is_south_west(direction):
        return direction in {6, 7, 8}
    
    @staticmethod
    def m_is_south_east(direction):
        return direction in {2, 3, 4}
    
    @staticmethod
    def m_ball_undetected(direction):
        return direction == 0
    
    def m_should_robot_stop(self, front_angle, back_angle):
        return self.m_ball_front(front_angle) or self.m_ball_behind(back_angle)

    
    def m_face_ball(self, speed):
        """
        aneurysm methods
        """
        #i pray to god this works
        #i might have an aneurysm
        while True:
            f_angle, f_strength, b_angle, b_strength = self.m_sensor.m_read()
            print((f_angle, f_strength, b_angle, b_strength))
            if f_angle != k_CENTER_ANGLE:
                self.m_motors.m_drive_vector(0, 0, speed)
            else:
                self.m_motors.m_stop_all()
                print("Ball is in the front")
                break
    def m_kick_ball_and_stop(self, speed):
        #this literally kicks the ball and then stops
        self.m_motors.m_drive_vector(0, speed)
        while True:
            f_angle, f_strength, b_angle, b_strength = self.m_sensor.m_read()
            if self.m_should_robot_stop(f_angle, b_angle):
                self.m_motors.m_stop_all()
                break
    async def m_main(self):
        """
        gg it's joever
        """
        #gg
        k = 80
        vy = 1000
        while True:
            f_angle, f_strength, b_angle, b_strength = self.m_sensor.m_read()
            delta_front = f_angle - k_CENTER_ANGLE
            delta_back = b_angle - k_CENTER_ANGLE
            print(f_angle, b_angle)
            if f_angle != 0 and b_angle in {0, 4, 8, 12, 16, 20, 24, 28, 32, 36}:
                print("front sensor activated")
                vx = delta_front * k
                if f_angle in {16, 20, 24}:
                    self.m_motors.m_drive_vector(0, vy)
                elif f_angle in {4, 32}:
                    self.m_motors.m_drive_vector(0, -vy)
                else:
                    self.m_motors.m_drive_vector(vx, vy)
            elif b_angle != 0 and f_angle == 0:
                print("back sensor activated")
                vx = delta_back * k
                print("vx = ", vx)
                if b_angle == 20:
                    self.m_motors.m_drive_vector(vy, 0)
                elif b_angle in {4, 36}:
                    self.m_motors.m_drive_vector(0, -vy)
                else:
                    self.m_motors.m_drive_vector(0, -vy)
            else:
                self.m_motors.m_stop_all()
if __name__ == "__main__":
    runloop.run(Robot().m_main())
