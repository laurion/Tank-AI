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
class CCDTest (Framework):
    name="CCDTest"
    def __init__(self):
        super(CCDTest, self).__init__()
        # Note that this test has a lot of commented-out versions that aren't implemented here.
        # See the original source code for more (Box2D/Examples/TestBed/Tests/CCDTest.h)
        k_restitution = 1.4

        bd=box2d.b2BodyDef()
        bd.position = (0.0, 20.0)
        body = self.world.CreateBody(bd) 

        sd=box2d.b2PolygonDef() 
        sd.density = 0.0
        sd.restitution = k_restitution

        sd.SetAsBox(0.1, 10.0, (-10.0, 0.0), 0.0)
        body.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (10.0, 0.0), 0.0)
        body.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (0.0, -10.0), 0.5 * box2d.b2_pi)
        body.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (0.0, 10.0), -0.5 * box2d.b2_pi)
        body.CreateShape(sd)

        sd_bottom=box2d.b2PolygonDef() 
        sd_bottom.SetAsBox( 1.5, 0.15 )
        sd_bottom.density = 4.0

        sd_left=box2d.b2PolygonDef() 
        sd_left.SetAsBox(0.15, 2.7, (-1.45, 2.35), 0.2)
        sd_left.density = 4.0

        sd_right=box2d.b2PolygonDef() 
        sd_right.SetAsBox(0.15, 2.7, (1.45, 2.35), -0.2)
        sd_right.density = 4.0

        bd=box2d.b2BodyDef() 
        bd.position = ( 0.0, 15.0 )
        body = self.world.CreateBody(bd) 
        body.CreateShape(sd_bottom)
        body.CreateShape(sd_left)
        body.CreateShape(sd_right)
        body.SetMassFromShapes()
        
        return

        for i in range(0):
            bd=box2d.b2BodyDef() 
            bd.position = (0.0, 15.0 + i)
            bd.isBullet = True
            body = self.world.CreateBody(bd) 
            body.SetAngularVelocity(box2d.b2Random(-50.0, 50.0))

            sd=box2d.b2CircleDef() 
            sd.radius = 0.25
            sd.density = 1.0
            sd.restitution = 0.0
            body.CreateShape(sd)
            body.SetMassFromShapes()


if __name__=="__main__":
     main(CCDTest)
