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
class DistanceTest (Framework):
    name="DistanceTest"
    body1=None
    body2=None
    shape1=None
    shape2=None
    _pickle_vars = ['body1', 'body2', 'shape1', 'shape2']
    def __init__(self):
        super(DistanceTest, self).__init__()
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(1.0, 1.0)
        sd.density = 0.0

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, 10.0)
        self.body1 = self.world.CreateBody(bd)
        self.shape1 = self.body1.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.vertexCount = 3
        sd.setVertex(0,-1.0, 0.0)
        sd.setVertex(1,1.0, 0.0)
        sd.setVertex(2,0.0, 15.0)
        sd.density = 1.0
        bd=box2d.b2BodyDef() 
        #bd.position = (-48.377853, 0.49244255)
        #bd.rotation = 90.475891
        bd.position = (0.0, 10.0)
        self.body2 = self.world.CreateBody(bd)
        self.shape2 = self.body2.CreateShape(sd)
        self.body2.SetMassFromShapes()

        self.world.SetGravity((0.0, 0.0))
     
    def Step(self, settings):
        if self.body2:
            distance, x1, x2 = box2d.b2Distance(self.shape1, self.body1.GetXForm(), self.shape2, self.body2.GetXForm())

            self.DrawStringCR("distance = %g" % distance)

            self.debugDraw.DrawPoint(x1, settings.pointSize, box2d.b2Color(1.0, 0.0, 0.0))
            self.debugDraw.DrawPoint(x2, settings.pointSize, box2d.b2Color(1.0, 0.0, 0.0))
            self.debugDraw.DrawSegment(x1, x2, box2d.b2Color(1.0, 1.0, 0.0))

        settings.pause = True
        super(DistanceTest, self).Step(settings)
        settings.pause = False
     
    def Keyboard(self, key):
        if not self.body2:
            return

        p = self.body2.GetPosition()
        a = self.body2.GetAngle()

        if key==K_a:
           p.x -= 0.1
        elif key==K_d:
           p.x += 0.1
        elif key==K_s:
           p.y -= 0.1
        elif key==K_w:
           p.y += 0.1
        elif key==K_q:
           a += 0.1 * box2d.b2_pi
        elif key==K_e:
           a -= 0.1 * box2d.b2_pi
        self.body2.SetXForm(p, a)
     
if __name__=="__main__":
     main(DistanceTest)
