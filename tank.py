import os
import random
from time import sleep

from utilities import defaultNonedict

import conf


class Tank(object):
    def __init__(self, name):
        self.name = name

        self._overtime_count = 0

        self.sensors = defaultNonedict()

        self._steps = []

        self._err = False
        self._finished = False
        self.force(0)
        self.torque(0)
        self._fire = '_'
        self._ping = 0
        self._turretangle = 0

        self.logfile = None


    def initialize(self):
        ''# Is called when the tank is created (must return in less than 1 second)

    def respond(self):
        # Is called 60 times per second (must return in less than 0.015 seconds)
        # The sensors can be accessed through self.sensprs
        # To control, us the tank.Tank class methods
        self.force(0)
        self.torque(0)

        if self._steps:
            nextstep = self._steps[0]
            steps = nextstep[0]
            f = nextstep[1]
            p = nextstep[2]

            if steps == 0:
                
                pass
            elif steps > 1:
                nextstep[0] -= 1
            else:
                
                self._steps.pop(0)

            
            
            f(*p)

    def forsteps(self, steps, f, *p):
        self._steps.append([steps, f, p])

    def forseconds(self, seconds, f, *p):
        steps = int(seconds*60.0)
        self.forsteps(steps, f, *p)

    def forever(self, f, *p):
        self.forsteps(0, f, *p)

    def err(self):
        self._err = True

    def finished(self):
        self._finished = True

    def force(self, n):
        self._force = int(n)

    def torque(self, n):
        self._torque = int(n)

    def fire(self, dist=None):

        if dist is None:
            self._fire = 'X'
        else:
            self._fire = int(dist)

    def ping(self):
        self._ping = 1

    def turret(self, angle):
        self._turretangle = angle

    def log(self, *msgs):
        if self.logfile is not None:
            msgstrs = map(str, msgs)
            m = ', '.join(msgstrs)
            msg = '%s: %s' % (self.name, m)
            self.logfile.write(msg)
            self.logfile.write('\n')
            self.logfile.flush()

    @property
    def response(self):
        if self._err:
            return 'ERROR'
        if self._finished:
            return 'END'
        else:
            r = 'FORCE:%s|TORQUE:%s|FIRE:%s|PING:%s|TURRET:%s' % (self._force,
                                                    self._torque,
                                                    self._fire, self._ping,
                                                    self._turretangle)
            self._fire = '_'
            self._ping = 0

            return r

    def test005(self):
        sleep(0.005)

    def test01(self):
        sleep(0.01)

    def test01a(self):
        sleep(0.01001)

    def test1(self):
        sleep(0.1)

    def test2(self):
        sleep(0.2)

    def testrand(self):
        t = random.randrange(1, 10) / 1000.
        sleep(t)
