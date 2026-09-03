
import runloop
import motor
import color_sensor
from hub import port

LEFT_PORT, RIGHT_PORT, FRONT_PORT, BACK_PORT, SENSOR_PORT, WALL_PORT = port.E, port.A, port.F, port.B, port.C, port.D
class Sensor:
    """Sensor wrapper for reading angles and strengths."""

    def __init__(self, port_=SENSOR_PORT):
        self.port = port_

    @staticmethod
    def _dec(v):
        return v // 4

    def read(self):
        # Reflection is its own distinct reading, separate from the RGB channels
        reflection = color_sensor.reflection(self.port)
        # color_sensor.rgbi() returns (red, green, blue, intensity)
        r, g, b, _ = color_sensor.rgbi(self.port)
        data = (self._dec(reflection), r, self._dec(b), g)
        print("SENSOR READ:\n{}".format(data))
        return data


class MotorController:
    """Encapsulates motor port operations and movement primitives."""

    def __init__(self):
        self.ports = (LEFT_PORT, RIGHT_PORT, FRONT_PORT, BACK_PORT)

    def apply(self, speeds):
        for p, spd in zip(self.ports, speeds):
            print("MOTOR RUN:\nmotor={} speed={}".format(p, spd))
            motor.run(p, spd)

    def stop(self):
        for p in self.ports:
            motor.stop(p)


class Robot:
    """High-level robot behavior combining sensors and motors."""

    DIR = dict(front=5, behind=1, dead_left=0, dead_right=2,
            nw={1, 2}, ne={2, 3}, sw={2, 3}, se={1, 2}, none=0)

    MOVE = {
        "fwd":lambda s: (-s,s,0,0),
        "back":lambda s: ( s, -s,0,0),
        "left":lambda s: ( 0,0,s, -s),
        "right": lambda s: ( 0,0, -s,s),
        "nw":    lambda s: (-s,s,s, -s),
        "ne":    lambda s: ( s, -s, -s,s),
        "sw":    lambda s: ( s, -s,s, -s),
        "se":    lambda s: (-s,s, -s,s),
        "rot":lambda s: ( s,s,s,s),
        "stop":lambda s: ( 0,0,0,0),
    }

    def __init__(self):
        self.s, self.m = Sensor(), MotorController()
        self.k, self.vy, self.state = 80, 1000, "idle"

    def _go(self, name, speed=0):
        self.m.apply(self.MOVE[name](speed))

    def should_robot_stop(self, f, b):
        return f == self.DIR["front"] or b == self.DIR["behind"]

    async def face_ball(self, speed):
        while True:
            f, _, b, _ = self.s.read()
            print((f, b))
            if f != 20:
                self._go("rot", speed)
            else:
                self._go("stop")
                print("Ball is in the front")
                break
            await runloop.sleep_ms(10)

    async def kick_ball_and_stop(self, speed):
        self._go("fwd", speed)
        while True:
            f, _, b, _ = self.s.read()
            if self.should_robot_stop(f, b):
                self._go("stop")
                break
            await runloop.sleep_ms(10)

    def _front(self, f):
        print("front sensor activated")
        vx = (f - 5) * self.k
        if f in {4, 5, 6}:
            self.m.apply((0, 0, 0, 0))
            self._go("fwd", self.vy)
        elif f in {1, 8}:
            self.m.apply((0, 0, 0, 0))
            self._go("back", self.vy)
        else:
            self._go("fwd", self.vy)
            self._go("left", vx)

    def _back(self, b):
        print("back sensor activated")
        vx = (b - 5) * self.k
        print("vx = ", vx)
        if b == 5:
            self.m.apply((0, 0, 0, 0))
            self._go("left", self.vy*10000)
        else:
            self.m.apply((0, 0, 0, 0))
            self._go("back", self.vy*10000)

            # An else statement should be used here
            # So if none of the conditions above were filled (aka robot went past the midpoint line) start moving back
            # E.g motor_run backwards

    async def main(self):
        handlers = {"front": self._front, "back": self._back}
        while True:
            f, _, b, _ = self.s.read()
            print(f, b)
            if f and b in range(10):# Probably add extra conditions here, e.g. Distance_Sensor <= x cm
                self.state = "front"
            elif b and not f:# Here as well Distance_Sensor <= x cm
                self.state = "back"
            else:
                self.state = "idle"
            if self.state in handlers:
                handlers[self.state](f if self.state == "front" else b)
            await runloop.sleep_ms(10)


runloop.run(Robot().main())