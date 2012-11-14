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

class LineJoint (Framework):
    name="LineJoint"
    def __init__(self):
        super(LineJoint, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)
    
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(0.5, 2.0)
        sd.density = 1.0
        
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, 7.0)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        body.SetMassFromShapes()
        
        jd = box2d.b2LineJointDef()
        axis=box2d.b2Vec2(2.0, 1.0)
        axis.Normalize()
        jd.Initialize(ground, body, (0.0, 8.5), axis)
        jd.motorSpeed = 0.0
        jd.maxMotorForce = 100.0
        jd.enableMotor = True
        jd.lowerTranslation = -4.0
        jd.upperTranslation = 4.0
        jd.enableLimit = True
        self.world.CreateJoint(jd)
    
if __name__=="__main__":
    main(LineJoint)
