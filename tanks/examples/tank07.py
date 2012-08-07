from tank import Tank

class TheTank(Tank):
    def initialize(self):
        self.health = 100

        self._turnto = 5

        self._moveto_choices = [70, 70, -70, -70]

    def respond(self):
        self.scan_and_fire()

        
        health = self.sensors['HEALTH']
        if health != self.health:
            self.health = health
            self._turnto += 1

        self.turnto()
        self.moveto()

    def turnto(self):
        

        turnto = 90 * self._turnto

        gyro = self.sensors['GYRO']

        gain = 2.5
        error = gyro - turnto
        torque = -gain * error

        self.torque(torque)

    def moveto(self):
        

        moveto = self._moveto_choices[self._turnto%4]

        pos = self.sensors['POS']

        gain = 6
        coord = pos[self._turnto%2]
        sign = [-1, -1, 1, 1][self._turnto%4]
        error = coord - moveto
        force = max(min(10, sign * gain * error), -10)

        self.log(pos, sign, coord, moveto, force, self._turnto, self.sensors['GYRO'])

        self.force(force)

    def scan_and_fire(self):
        

        tur = self.sensors['TUR']
        self.turret(tur-20)
        self.ping()

        kind, angle, dist = self.sensors['PING']
        if kind in 't':
            if dist > 4:
                
                self.fire(dist)
            else:
                self.fire()
