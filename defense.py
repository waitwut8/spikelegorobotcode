"""
Pybricks-based defensive robot implementing two scenarios:

Scenario 1: face the ball and push it forward until it passes a midpoint
           or until the ball approaches the wall (switch to scenario 2).
Scenario 2: move slowly toward the ball; if the ball crosses back across
           the midpoint (i.e. retreats past the midpoint), retreat.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait


# Ports (adjust to your robot)
LEFT_PORT = Port.F
RIGHT_PORT = Port.B
COLOR_PORT = Port.C
DISTANCE_PORT = Port.D


class DefensiveRobot:
    MIDPOINT_DISTANCE = 300    # mm ball must move to be considered past midpoint
    WALL_DISTANCE_THRESHOLD = 150  # mm - when close to wall, enter scenario 2
    BALL_CLOSE_DISTANCE = 50       # mm - (unused for retreat; kept for tuning)
    PUSH_SPEED = 600               # deg/s for push
    SLOW_SPEED = 250               # deg/s for slow approach
    TURN_SPEED = 150               # deg/s for turning while aiming

    def __init__(self):
        self.hub = PrimeHub()
        self.left = Motor(LEFT_PORT)
        self.right = Motor(RIGHT_PORT)
        self.color = ColorSensor(COLOR_PORT)
        self.ultra = UltrasonicSensor(DISTANCE_PORT)

        self.current_scenario = 1
        self.ball_initial_distance = None

    def get_distance(self):
        try:
            return self.ultra.distance()
        except Exception:
            return None

    def get_reflection(self):
        try:
            return self.color.reflection()
        except Exception:
            return None

    # Basic motor helpers (differential drive assumed)
    def drive_forward(self, speed):
        self.left.run(speed)
        self.right.run(speed)

    def drive_backward(self, speed):
        self.left.run(-speed)
        self.right.run(-speed)

    def turn_left(self, speed):
        self.left.run(-speed)
        self.right.run(speed)

    def turn_right(self, speed):
        self.left.run(speed)
        self.right.run(-speed)

    def stop(self):
        self.left.stop()
        self.right.stop()

    def aim_at_ball(self):
        """Simple aiming using color reflection as lateral cue.
        Returns True when roughly centered.
        """
        reflection = self.get_reflection()
        if reflection is None:
            return False

        # Assumed center reflection; adjust as needed for your sensor/ball
        target = 50
        error = reflection - target
        deadband = 6

        if abs(error) <= deadband:
            self.stop()
            return True

        # Turn proportionally (simple bang-bang for clarity)
        if error > 0:
            self.turn_right(self.TURN_SPEED)
        else:
            self.turn_left(self.TURN_SPEED)

        return False

    def scenario_1_push_ball(self):
        dist = self.get_distance()
        if dist is None:
            print("[S1] No distance reading")
            return

        if self.ball_initial_distance is None:
            self.ball_initial_distance = dist
            print(f"[S1] initial ball distance {dist}mm")

        traveled = self.ball_initial_distance - dist
        print(f"[S1] ball dist={dist} traveled={traveled}")

        # Switch to scenario 2 if ball is too close to wall
        if dist < self.WALL_DISTANCE_THRESHOLD:
            print("[S1] near wall -> switching to Scenario 2")
            self.current_scenario = 2
            self.ball_initial_distance = None
            self.stop()
            return

        # Aim first, then push
        if not self.aim_at_ball():
            return

        # Push forward
        print("[S1] pushing ball")
        self.drive_forward(self.PUSH_SPEED)

    def scenario_2_defend_goal(self):
        dist = self.get_distance()
        if dist is None:
            print("[S2] No distance reading")
            return

        if self.ball_initial_distance is None:
            self.ball_initial_distance = dist
            print(f"[S2] entry distance {dist}mm")

        traveled = self.ball_initial_distance - dist
        print(f"[S2] ball dist={dist} traveled={traveled}")

        # If ball has crossed back across the midpoint (i.e. negative travel past midpoint), retreat
        if traveled < -self.MIDPOINT_DISTANCE:
            print("[S2] ball crossed midpoint backward -> retreat")
            self.drive_backward(self.SLOW_SPEED)
            return

        # Aim at ball then slowly approach
        if not self.aim_at_ball():
            return

        print("[S2] moving slowly toward ball")
        self.drive_forward(self.SLOW_SPEED)

        # If ball moves far away from goal, return to scenario 1
        if dist > self.WALL_DISTANCE_THRESHOLD + 100:
            print("[S2] ball moved away -> back to Scenario 1")
            self.current_scenario = 1
            self.ball_initial_distance = None

    def update(self):
        try:
            if self.current_scenario == 1:
                self.scenario_1_push_ball()
            else:
                self.scenario_2_defend_goal()
        except Exception as e:
            print("[ERROR]", e)
            self.stop()


def main():
    r = DefensiveRobot()
    try:
        while True:
            r.update()
            wait(100)
    except KeyboardInterrupt:
        r.stop()


if __name__ == "__main__":
    main()