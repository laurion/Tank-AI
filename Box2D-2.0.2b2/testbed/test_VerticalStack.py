#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
# Python version Copyright (c) 2008 kne / sirkne at gmail dot com
# 
# Implemented using the pybox2d SWIG interface for Box2D (pybox2d.googlecode.com)
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

from test_main import *
class VerticalStack (Framework):
    name="VerticalStack"
    bullet=None
    _pickle_vars=['bullet']
    def __init__(self):
        super(VerticalStack, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0, (0.0, -10.0), 0.0)

        bd=box2d.b2BodyDef()
        bd.position = (0.0, 0.0)
        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (20.0, 10.0), 0.0)
        ground.CreateShape(sd)

        xs = [0.0, -10.0, -5.0, 5.0, 10.0]

        for j in xrange(5):
            sd=box2d.b2PolygonDef()
            sd.SetAsBox(0.5, 0.5)
            sd.density = 1.0
            sd.friction = 0.3

            for i in xrange(16):
                bd=box2d.b2BodyDef()

                x = 0.0
                #x = b2Random(-0.1, 0.1)
                #x = i % 2 == 0 ? -0.025 : 0.025
                bd.position = (xs[j] + x, 0.752 + 1.54 * i)
                body = self.world.CreateBody(bd)

                body.CreateShape(sd)
                body.SetMassFromShapes()

        self.bullet = None 
     
    def Keyboard(self, key):
        if key == K_COMMA:
            if self.bullet:
                self.world.DestroyBody(self.bullet)
                self.bullet = None

            sd=box2d.b2CircleDef()
            sd.density = 20.0
            sd.radius = 0.25
            sd.restitution = 0.05

            bd=box2d.b2BodyDef()
            bd.isBullet = True
            bd.position = (-31.0, 5.0)

            self.bullet = self.world.CreateBody(bd)
            self.bullet.CreateShape(sd)
            self.bullet.SetMassFromShapes()

            self.bullet.SetLinearVelocity((400.0, 0.0))
     
    def Step(self, settings):
          self.DrawStringCR("Press: (,) to launch a bullet.")

          super(VerticalStack, self).Step(settings)

if __name__=="__main__":
     main(VerticalStack)
