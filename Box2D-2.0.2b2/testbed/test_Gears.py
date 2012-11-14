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

class Gears (Framework):
    name="Gears"
    joint1=None
    joint2=None
    joint3=None
    joint4=None
    joint5=None
    _pickle_vars = ['joint1', 'joint2', 'joint3',  'joint4',  'joint5']
    def __init__(self):
        super(Gears, self).__init__()
        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 10.0)
        ground.CreateShape(sd)

        circle1=box2d.b2CircleDef() 
        circle1.radius = 1.0
        circle1.density = 5.0

        circle2=box2d.b2CircleDef() 
        circle2.radius = 2.0
        circle2.density = 5.0

        box=box2d.b2PolygonDef() 
        box.SetAsBox(0.5, 5.0)
        box.density = 5.0

        bd1=box2d.b2BodyDef() 
        bd1.position = (-3.0, 12.0)
        body1 = self.world.CreateBody(bd1) 
        body1.CreateShape(circle1)
        body1.SetMassFromShapes()

        jd1=box2d.b2RevoluteJointDef() 
        jd1.body1 = ground
        jd1.body2 = body1
        jd1.localAnchor1 = ground.GetLocalPoint(bd1.position)
        jd1.localAnchor2 = body1.GetLocalPoint(bd1.position)
        jd1.referenceAngle = body1.GetAngle() - ground.GetAngle()
        self.joint1 = self.world.CreateJoint(jd1)

        bd2=box2d.b2BodyDef() 
        bd2.position = (0.0, 12.0)
        body2 = self.world.CreateBody(bd2) 
        body2.CreateShape(circle2)
        body2.SetMassFromShapes()

        jd2=box2d.b2RevoluteJointDef() 
        jd2.Initialize(ground, body2, bd2.position)
        self.joint2 = self.world.CreateJoint(jd2)

        bd3=box2d.b2BodyDef() 
        bd3.position = (2.5, 12.0)
        body3 = self.world.CreateBody(bd3) 
        body3.CreateShape(box)
        body3.SetMassFromShapes()

        jd3=box2d.b2PrismaticJointDef() 
        jd3.Initialize(ground, body3, bd3.position, (0.0, 1.0))
        jd3.lowerTranslation = -5.0
        jd3.upperTranslation = 5.0
        jd3.enableLimit = True

        self.joint3 = self.world.CreateJoint(jd3)

        jd4=box2d.b2GearJointDef() 
        jd4.body1 = body1
        jd4.body2 = body2
        jd4.joint1 = self.joint1
        jd4.joint2 = self.joint2
        jd4.ratio = circle2.radius / circle1.radius
        self.joint4 = self.world.CreateJoint(jd4)

        jd5=box2d.b2GearJointDef() 
        jd5.body1 = body2
        jd5.body2 = body3
        jd5.joint1 = self.joint2
        jd5.joint2 = self.joint3
        jd5.ratio = -1.0 / circle2.radius
        self.joint5 = self.world.CreateJoint(jd5)
             
    def Step(self, settings):
        if self.joint1 and self.joint2 and self.joint3 and self.joint4 and self.joint5:
            ratio = self.joint4.GetRatio()
            value = self.joint1.GetJointAngle() + ratio * self.joint2.GetJointAngle()
            self.DrawStringCR("theta1 + %.2f * theta2 = %.2f" % (ratio, value))

            ratio = self.joint5.GetRatio()
            value = self.joint2.GetJointAngle() + ratio * self.joint3.GetJointTranslation()
            self.DrawStringCR("theta2 + %.2f * delta = %.2f" % (ratio, value))

        super(Gears, self).Step(settings)

if __name__=="__main__":
     main(Gears)
