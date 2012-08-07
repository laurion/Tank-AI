import pygame

from pygsear.Game import Game
from pygsear.Drawable import RotatedImage, Square, Rectangle, Stationary, Image, String, Circle
from pygsear.Widget import ProgressBar
from pygsear.locals import RED, ORANGE, YELLOW

import pygsear.conf
pygsear.conf.MAX_FPS = 60

m = 15 
size = 2 * m

import conf


def trans(pos):
    try:
        px, py =  pos.x, pos.y
    except AttributeError:
        px, py = pos[0], pos[1]
    sz = size / 2
    x, y = (px*sz)+300, (py*sz)+300
    return x, y

def scale(s):
    w, h = s
    sw, sh = (w*size), (h*size)
    return sw, sh

class Tank(RotatedImage):
    ntanks = 0
    def __init__(self, pos, ang):
        Tank.ntanks += 1
        filename = 't{0:02d}.png'.format(Tank.ntanks)
        steps = 360
        RotatedImage.__init__(self, filename=filename, steps=steps)
        if size != 30:
            self.stretch(size=(size, size))

        self.turr = Turret(pos, 0)

        self.setpos(pos)
        self.set_rotation(ang)

    def setpos(self, pos):
        x, y = trans(pos)
        self.set_position(x, y)
        self.turr.set_position(x, y)

    def set_turr_rot(self, ang):
        self.turr.set_rotation(self.rotation+ang)

    def kill(self):
        self.turr.kill()
        RotatedImage.kill(self)

class Turret(RotatedImage):
    def __init__(self, pos, ang):
        filename = 'turret.png'
        steps = 360
        RotatedImage.__init__(self, filename=filename, steps=steps)
        
        


class HealthBar(ProgressBar):
    def __init__(self, n):
        ProgressBar.__init__(self, width=150, steps=conf.maxhealth,
                                position=(640, (60*n+34)), color=(100, 100, 100))

    def step(self, steps):
        self.stepsLeft -= steps-1
        ProgressBar.step(self)
        if self.stepsLeft <= 0.30 * self.steps:
            self.set_color(RED)
            self.show()

class TankInfo(object):
    def __init__(self, n, name):
        self.n = n

        filename = 't{0:02d}.png'.format(n)
        iconsprite = Image(filename=filename)
        self.icon = Stationary(sprite=iconsprite)
        self.icon.set_position((630, (60*n+0)))
        self.icon.draw()
        health = HealthBar(n)
        self.health = health

class Bullet(Square):
    def __init__(self, pos):
        Square.__init__(self, side=3,color=(0, 0, 0))
        
        self.setpos(pos)

    def setpos(self, pos):
        x, y = trans(pos)
        self.set_position(x, y)

class Explosion(Circle):
    def __init__(self, pos):
        r = conf.explosion_radii[-1] 
        rt = r * m
        Circle.__init__(self, radius=rt)
        self.setpos(pos)

    def setpos(self, pos):
        x, y = trans(pos)
        self.set_position(x-45, y-45)

    def paint(self):
        image = self.image
        x, y = self.radius, self.radius
        r3 = zip(conf.explosion_radii, (RED, ORANGE, YELLOW))
        for r, color in r3:
            rt = m * r
            pygame.draw.circle(image, color, (x, y), rt, 2)


class Wall(Stationary):
    def __init__(self, pos, size):
        cx, cy = trans(pos)
        w, h = scale(size)
        x = cx-(w/2)
        y = cy-(h/2)

        sq = Rectangle(width=w, height=h, color=(150, 210, 200))
        Stationary.__init__(self, sprite=sq)
        self.set_position((x, y))
        self.draw()

class Arena(Game):
    splash_filename = 'splashscreen.png'
    def __init__(self):
        Game.__init__(self)
        self.set_background(color=(255, 255, 255))
        self.set_title('Tank AI')
        titleicon = String(message="Tank AI", fontSize=40, color=(50, 50, 50))
        title = Stationary(sprite=titleicon)
        title.set_position((660, 10))
        title.draw()

    def addtank(self, pos, ang):
        v = Tank(pos, ang)
        self.sprites.add(v)
        self.sprites.add(v.turr, level=1)
        return v

    def addtankinfo(self, n, name):
        i = TankInfo(n, name)
        self.sprites.add(i.health)
        return i

    def addbullet(self, pos):
        v = Bullet(pos)
        self.sprites.add(v)
        return v

    def addexplosion(self, pos):
        e = Explosion(pos)
        self.sprites.add(e)
        return e
