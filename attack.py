from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, Stop
from pybricks.robotics import run_task

LMP = Port.A
RMP = Port.B
FMP = Port.C
BMP = Port.D

LDIR = Direction.CLOCKWISE
RDIR = Direction.CLOCKWISE
FDIR = Direction.CLOCKWISE
BDIR = Direction.CLOCKWISE

DS = 600
TG = 8
VFS = {18, 19, 20, 21, 22}


class MR:
    def __init__(self):
        self.lm = Motor(LMP, LDIR)
        self.rm = Motor(RMP, RDIR)
        self.fm = Motor(FMP, FDIR)
        self.bm = Motor(BMP, BDIR)

        self.ds = DS
        self.tg = TG
        self.vfs = VFS

    def fa(self):
        return 0

    def fs(self):
        return 0

    def ba(self):
        return 0

    def bs(self):
        return 0

    def rd(self):
        r = (self.fa(), self.fs(), self.ba(), self.bs())
        print("DEBUG: Sensor reading:", r)
        return r

    def rl(self, s):
        print("MOTOR RUN:\nleft_motor={} speed={}".format(self.lm, s))
        self.lm.run(s)

    def rr(self, s):
        print("MOTOR RUN:\nright_motor={} speed={}".format(self.rm, s))
        self.rm.run(s)

    def rf(self, s):
        print("MOTOR RUN:\nfront_motor={} speed={}".format(self.fm, s))
        self.fm.run(s)

    def rb(self, s):
        print("MOTOR RUN:\nback_motor={} speed={}".format(self.bm, s))
        self.bm.run(s)

    def stop(self):
        self.lm.stop()
        self.rm.stop()
        self.fm.stop()
        self.bm.stop()

    def gft(self, s):
        print("Robot moving forward with speed={}".format(s))
        self.lm.run_time(-1000, 2000, Stop.BRAKE)
        self.rm.run_time(1000, 2000, Stop.BRAKE)

    def glt(self, s):
        print("Robot moving left with speed={}".format(s))
        self.fm.run_time(-1000, 2000, Stop.BRAKE)
        self.bm.run_time(1000, 2000, Stop.BRAKE)

    def gf(self, s):
        print("Robot moving forward with speed={}".format(s))
        self.rl(-s)
        self.rr(s)

    def gb(self, s):
        print("Robot moving backwards with speed={}".format(s))
        self.rl(s)
        self.rr(-s)

    def gl(self, s):
        print("Robot moving left with speed={}".format(s))
        self.rf(s)
        self.rb(-s)

    def gr(self, s):
        print("Robot moving right with speed={}".format(s))
        self.rf(-s)
        self.rb(s)

    def gnw(self, s):
        self.rf(s)
        self.rb(-s)
        self.rl(-s)
        self.rr(s)

    def gne(self, s):
        self.rf(-s)
        self.rb(s)
        self.rl(s)
        self.rr(-s)

    def gsw(self, s):
        self.rf(s)
        self.rb(-s)
        self.rl(s)
        self.rr(-s)

    def gse(self, s):
        self.rf(-s)
        self.rb(s)
        self.rl(-s)
        self.rr(s)

    def rot(self, s):
        print("Robot rotating with speed={}".format(s))
        self.rl(s)
        self.rr(s)
        self.rb(s)
        self.rf(s)

    @staticmethod
    def bf(d):
        return d in range(18, 22)

    @staticmethod
    def bb(d):
        return d in range(2, 6)

    @staticmethod
    def bdl(d):
        return d == 1

    @staticmethod
    def bdr(d):
        return d == 9

    @staticmethod
    def nw(d):
        return d in {2, 3, 4}

    @staticmethod
    def ne(d):
        return d in {6, 7, 8}

    @staticmethod
    def sw(d):
        return d in {6, 7, 8}

    @staticmethod
    def se(d):
        return d in {2, 3, 4}

    @staticmethod
    def bu(d):
        return d == 0

    def srs(self, f, b):
        return self.bf(f) or self.bb(b)

    def fb(self, s):
        while True:
            sig = self.rd()
            print(sig)
            if sig[0] != 20:
                self.rot(s)
            else:
                self.stop()
                print("Ball is in the front")
                break

    def kb(self, s):
        self.gf(s)
        while True:
            sig = self.rd()
            if self.srs(sig[0], sig[2]):
                self.stop()
                break

    async def main(self):
        while True:
            f, _, b, _ = self.rd()
            df = f - 20
            db = b - 20
            print(f, b)

            if f != 0 and b in self.vfs:
                print("front sensor activated")
                if f in {8, 12, 16}:
                    self.gl(self.ds)
                    self.gf(self.ds)
                elif f in {4}:
                    self.gb(self.ds)
                    self.gl(self.ds)
                elif f in {28} or b in {24}:
                    self.gb(self.ds)
                    self.gr(self.ds)
                elif f in {20, 24}:
                    self.gr(self.ds)
                    self.gf(self.ds)

            elif b != 0 and f == 0:
                print("back sensor activated")
                if b in {32, 28}:
                    self.gl(self.ds)
                    self.gb(self.ds)
                elif b in {36}:
                    self.gr(self.ds)
                    self.gb(self.ds)
                elif b in {4, 8, 12, 16}:
                    self.gb(self.ds)
                    self.gr(self.ds)
                else:
                    self.gb(self.ds)
                    self.gr(self.ds)

            await wait(10)


run_task(MR().main)