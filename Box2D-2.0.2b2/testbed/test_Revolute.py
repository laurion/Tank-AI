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
class Revolute (Framework):
    name="Revolute"
    joint=None
    _pickle_vars=['joint']
    def __init__(self):
        super(Revolute, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)
    
        sd=box2d.b2CircleDef()
        sd.radius = 0.5
        sd.density = 5.0
        
        bd=box2d.b2BodyDef()
        
        rjd=box2d.b2RevoluteJointDef()
        
        bd.position = (0.0, 20.0)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        body.SetMassFromShapes()
        
        w = 100.0
        body.SetAngularVelocity(w)
        body.SetLinearVelocity((-8.0 * w, 0.0))
        
        rjd.Initialize(ground, body, (0.0, 12.0))
        rjd.motorSpeed = 1.0 * box2d.b2_pi
        rjd.maxMotorTorque = 10000.0
        rjd.enableMotor = False
        rjd.lowerAngle = -0.25 * box2d.b2_pi
        rjd.upperAngle = 0.5 * box2d.b2_pi
        rjd.enableLimit = True
        rjd.collideConnected = True
        
        self.joint = self.world.CreateJoint(rjd)
    
    def Keyboard(self, key):
        if not self.joint:
            return

        if key==K_l:
            self.joint.EnableLimit(not self.joint.IsLimitEnabled())
        elif key==K_s:
            self.joint.EnableMotor(False)
    
    def Step(self, settings):
        super(Revolute, self).Step(settings)
        self.DrawStringCR("Keys: (l) toggle limits, (s) motor off")
    
if __name__=="__main__":
    main(Revolute)
