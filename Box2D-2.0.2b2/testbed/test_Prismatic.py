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
class Prismatic (Framework):
    name="Prismatic"
    _pickle_vars=['joint']
    def __init__(self):
        super(Prismatic, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)
    
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(2.0, 0.5)
        sd.density = 5.0
        sd.friction = 0.05
        
        bd=box2d.b2BodyDef()
        bd.position = (-10.0, 10.0)
        bd.angle = 0.5 * box2d.b2_pi
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        body.SetMassFromShapes()
        
        pjd=box2d.b2PrismaticJointDef()
        
        # Bouncy limit
        pjd.Initialize(ground, body, (0.0, 0.0), (1.0, 0.0))
        
        # Non-bouncy limit
        #pjd.Initialize(ground, body, (-10.0, 10.0), (1.0, 0.0))
        
        pjd.motorSpeed = 10.0
        pjd.maxMotorForce = 1000.0
        pjd.enableMotor = True
        pjd.lowerTranslation = 0.0
        pjd.upperTranslation = 20.0
        pjd.enableLimit = True
        
        self.joint = self.world.CreateJoint(pjd)
    
    def Keyboard(self, key):
        if not self.joint:
            return

        if key==K_l:
            self.joint.EnableLimit(not self.joint.IsLimitEnabled())
        elif key==K_m:
            self.joint.EnableMotor(not self.joint.IsMotorEnabled())
        elif key==K_p:
            self.joint.SetMotorSpeed(-self.joint.GetMotorSpeed())
    
    def Step(self, settings):
        self.DrawStringCR("Keys: (l) limits, (m) motors, (p) speed")
        
        if self.joint:
            force = self.joint.GetMotorForce()
            self.DrawStringCR("Motor Force = %f.0" % force)

        super(Prismatic, self).Step(settings)
    
if __name__=="__main__":
    main(Prismatic)
