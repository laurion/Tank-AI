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

# A motor driven slider crank with joint friction.

class SliderCrank (Framework):
    name="SliderCrank"
    joint1=None
    joint2=None
    _pickle_vars=['joint1', 'joint2']

    def __init__(self):
        super(SliderCrank, self).__init__()
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 10.0)

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        # Define crank.
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.5, 2.0)
        sd.density = 1.0

        rjd=box2d.b2RevoluteJointDef() 

        prevBody=ground

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, 7.0)
        body = self.world.CreateBody(bd) 
        body.CreateShape(sd)
        body.SetMassFromShapes()

        rjd.Initialize(prevBody, body, (0.0, 5.0))
        rjd.motorSpeed = 1.0 * box2d.b2_pi
        rjd.maxMotorTorque = 10000.0
        rjd.enableMotor = True
        self.joint1 = self.world.CreateJoint(rjd)

        prevBody = body

        # Define follower.
        sd.SetAsBox(0.5, 4.0)
        bd.position = (0.0, 13.0)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        body.SetMassFromShapes()

        rjd.Initialize(prevBody, body, (0.0, 9.0))
        rjd.enableMotor = False
        self.world.CreateJoint(rjd)

        prevBody = body

        # Define piston
        sd.SetAsBox(1.5, 1.5)
        bd.position = (0.0, 17.0)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        body.SetMassFromShapes()

        rjd.Initialize(prevBody, body, (0.0, 17.0))
        self.world.CreateJoint(rjd)

        pjd=box2d.b2PrismaticJointDef() 
        pjd.Initialize(ground, body, (0.0, 17.0), (0.0, 1.0))

        pjd.maxMotorForce = 1000.0
        pjd.enableMotor = True

        self.joint2 = self.world.CreateJoint(pjd)

        # Create a payload
        sd.density = 2.0
        bd.position = (0.0, 23.0)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        body.SetMassFromShapes()

    def Keyboard(self, key):
        if not self.joint1 or not self.joint2:
            return

        if key==K_f:
            self.joint2.enableMotor = not self.joint2.enableMotor
            self.joint2.GetBody2().WakeUp()
           
        elif key==K_m:
            self.joint1.enableMotor = not self.joint1.enableMotor
            self.joint1.GetBody2().WakeUp()

    def Step(self, settings):
        self.DrawStringCR("Keys: (f) toggle friction, (m) toggle motor")

        if self.joint1:
            torque = self.joint1.GetMotorTorque()
            self.DrawStringCR("Motor Torque = %.0f" % (torque))

        super(SliderCrank, self).Step(settings)

if __name__=="__main__":
     main(SliderCrank)
