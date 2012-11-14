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
class Pulleys (Framework):
    name="Pulleys"
    joint1=None
    _pickle_vars=['joint1']
    def __init__(self):
        super(Pulleys, self).__init__()
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 10.0)

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        a = 2.0
        b = 4.0
        y = 16.0
        L = 12.0

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(a, b)
        sd.density = 5.0

        bd=box2d.b2BodyDef() 

        bd.position = (-10.0, y)
        body1 = self.world.CreateBody(bd) 
        body1.CreateShape(sd)
        body1.SetMassFromShapes()

        bd.position = (10.0, y)
        body2 = self.world.CreateBody(bd) 
        body2.CreateShape(sd)
        body2.SetMassFromShapes()

        pulleyDef=box2d.b2PulleyJointDef() 

        anchor1      =(-10.0, y + b)
        anchor2      =( 10.0, y + b)
        groundAnchor1=(-10.0, y + b + L)
        groundAnchor2=( 10.0, y + b + L)

        pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0)

        self.joint1 = self.world.CreateJoint(pulleyDef)
     
    def Step(self, settings):
        super(Pulleys, self).Step(settings)
        
        if not self.joint1:
            return

        ratio = self.joint1.GetRatio()
        L = self.joint1.GetLength1() + ratio * self.joint1.GetLength2()
        self.DrawStringCR("L1 + %.2f * L2 = %.2f" % (ratio, L))

if __name__=="__main__":
    main(Pulleys)
