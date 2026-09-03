# ══════════════════════════════════════════════════════════════════════════
# OMNI DRIVE CALIBRATION PROGRAM — SPIKE Prime / Pybricks
# Used for mechanical setup only
# ══════════════════════════════════════════════════════════════════════════

from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# Motor ports
LEFT_PORT = Port.F
RIGHT_PORT = Port.B
FRONT_PORT = Port.E
BACK_PORT = Port.A
SENSOR_PORT = Port.C

SPEED = 300


class Drive:
    def __init__(self):
        self.left = Motor(LEFT_PORT)
        self.right = Motor(RIGHT_PORT)
        self.front = Motor(FRONT_PORT)
        self.back = Motor(BACK_PORT)

    def stop(self):
        for m in (self.left, self.right, self.front, self.back):
            m.stop()

    def raw(self, a, b, c, d):
        """
        Direct motor test:
        left, right, front, back
        """
        self.left.run(a)
        self.right.run(b)
        self.front.run(c)
        self.back.run(d)

    def forward(self):
        self.raw(-SPEED, SPEED, 0, 0)

    def backward(self):
        self.raw(SPEED, -SPEED, 0, 0)

    def strafe_right(self):
        self.raw(0, 0, -SPEED, SPEED)

    def strafe_left(self):
        self.raw(0, 0, SPEED, -SPEED)

    def rotate_clockwise(self):
        self.raw(SPEED, SPEED, SPEED, SPEED)

    def rotate_counterclockwise(self):
        self.raw(-SPEED, -SPEED, -SPEED, -SPEED)


def test_motion(name, function):
    print("\nTEST:", name)
    print("Starting in 3 seconds")
    wait(3000)

    function()

    wait(2000)

    drive.stop()

    print("Finished:", name)
    wait(1000)


drive = Drive()


# ------------------------------------------------
# Run tests one at a time
# Comment out tests you don't need
# ------------------------------------------------

test_motion("FORWARD", drive.forward)

test_motion("BACKWARD", drive.backward)

test_motion("STRAFE RIGHT", drive.strafe_right)

test_motion("STRAFE LEFT", drive.strafe_left)

test_motion("ROTATE CLOCKWISE", drive.rotate_clockwise)

test_motion("ROTATE COUNTERCLOCKWISE",
            drive.rotate_counterclockwise)


drive.stop()

print("Calibration complete")