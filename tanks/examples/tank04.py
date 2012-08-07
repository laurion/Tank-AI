from tank import Tank

class TheTank(Tank):
    def initialize(self):
        
        self.forseconds(5, self.force, 50)
        self.forseconds(0.9, self.force, -10)
        self.forseconds(0.7, self.torque, 100)
        self.forseconds(6, self.force, 50)

        
        self.forever(self.scanfire)


        self._turretdirection = 1
        self.turret(180)
        self._pingfoundtank = None

    def scanfire(self):
        self.ping()

        sensors = self.sensors
        kind, angle, dist = sensors['PING']
        tur = sensors['TUR']

        if self._pingfoundtank is not None:
            
            if angle == self._pingfoundtank:
                
                if kind in 'rb':
                    
                    self.fire()
                else:
                    
                    self._pingfoundtank = None
            elif kind == 't':
                
                
                self.fire()
                self._pingfoundtank = angle
                self.turret(angle)
            else:
                
                
                self.turret(self._pingfoundtank)

        elif kind == 't':
            
            
            self.fire()
            
            self._pingfoundtank = angle
            self.turret(angle)

        else:
            
            
            if self._turretdirection == 1:
                if tur < 180:
                    self.turret(180)
                else:
                    self._turretdirection = 0
            elif self._turretdirection == 0:
                if tur > 90:
                    self.turret(90)
                else:
                    self._turretdirection = 1
