"""Compact robot controller for the 4-motor + single ColorSensor setup.

Sensor values are read as:
- front angle: reflection()
- front strength: raw red
- back angle: raw blue
- back strength: raw green
"""

import motor
from hub import motion_sensor, port
import runloop
import color_sensor

def log(level, message):
    print(f"[{level}] : {message}")


class MovementRobot:
    """Drive the robot using front/back seeker values and yaw correction."""

    L, R, F, B, S = port.F, port.B, port.E, port.A, port.C
    V = {0, 4, 8, 12, 16, 20, 24, 28, 32, 36}

    def __init__(self):
        self.t = 80
        self.v = 1000
        self.la = self.ac = self.h = 0
        self.s = 3
        self.r = False
        log("INFO", "MovementRobot initialized")

    def rs(self):
        """Return (front_angle, front_strength, back_angle, back_strength)."""
        f = color_sensor.reflection(self.S)
        c = color_sensor.rgbi(self.S)
        r = (f, c[0], c[2], c[1])
        log("DEBUG", f"Sensor reading: angle={r[0]}, front_strength={r[1]}, back_angle={r[2]}, back_strength={r[3]}")
        return r

    # Motor aliases: left/right are the drive pair, front/back are the strafe pair.
    def rl(self, v): motor.run(self.L, v)
    def rr(self, v): motor.run(self.R, v)
    def rf(self, v): motor.run(self.F, v)
    def rb(self, v): motor.run(self.B, v)

    def gf(self, v): log("INFO", f"Robot moving forward with speed={v}"); self.rl(-v); self.rr(v)
    def gb(self, v): log("INFO", f"Robot moving backwards with speed={v}"); self.rl(v); self.rr(-v)
    def gl(self, v): log("INFO", f"Robot moving left with speed={v}"); self.rf(v); self.rb(-v)
    def gr(self, v): log("INFO", f"Robot moving right with speed={v}"); self.rf(-v); self.rb(v)
    def st(self): self.rb(0); self.rf(0); self.rl(0); self.rr(0)
    def rot(self, v): self.rb(v); self.rf(v); self.rl(v); self.rr(v)

    def stable(self, a):
        """Require the same angle to be seen several times before acting."""
        if a == self.la:
            self.ac += 1
        else:
            self.la, self.ac = a, 0
        return self.ac >= self.s

    async def main(self):
        """Main control loop: yaw correction first, seeker tracking second."""
        motion_sensor.reset_yaw(0)
        await runloop.sleep_ms(250)
        log("INFO", "True North (0°) permanently set at startup")
        while True:
            y = motion_sensor.tilt_angles()[0] // 10
            e = -y
            f, _, b, _ = self.rs()
            print(f"Sensors: F={f} B={b} | Yaw={y} Error={e}")
            if abs(e) > 12:
                if not self.r:
                    self.st(); self.r = True; log("INFO", f"Starting rotation correction | error={e}")
                self.rot(max(-850, min(850, int(e * 22))))
            else:
                if self.r:
                    self.st(); self.r = False; log("INFO", "Heading stabilized, resuming sensor logic")
                if f and b in self.V:
                    if not self.stable(f):
                        await runloop.sleep_ms(8)
                        continue
                    print("front sensor activated")
                    if f in {8, 12, 16}:
                        self.gl(self.v); self.gf(self.v)
                    elif f == 4:
                        self.gb(self.v); self.gl(self.v)
                    elif f == 28 or b == 24:
                        self.gb(self.v); self.gr(self.v)
                    elif f in {20, 24}:
                        self.gr(self.v); self.gf(self.v)
                    else:
                        self.gr(self.v); self.gb(self.v)
                elif b and not f:
                    if not self.stable(b):
                        await runloop.sleep_ms(8)
                        continue
                    print("back sensor activated")
                    if b in {32, 28}:
                        self.gl(self.v); self.gb(self.v)
                    elif b == 36:
                        self.gr(self.v); self.gb(self.v)
                    else:
                        self.gb(self.v); self.gr(self.v)
            await runloop.sleep_ms(12)


runloop.run(MovementRobot().main())