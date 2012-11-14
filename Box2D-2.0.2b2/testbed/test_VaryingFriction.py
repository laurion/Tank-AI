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
class VaryingFriction (Framework):
    name="VaryingFriction"
     
    def __init__(self):
        super(VaryingFriction, self).__init__()
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 20.0)

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -20.0)
        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(13.0, 0.25)

        bd=box2d.b2BodyDef() 
        bd.position = (-4.0, 22.0)
        bd.angle = -0.25

        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.25, 1.0)

        bd=box2d.b2BodyDef() 
        bd.position = (10.5, 19.0)

        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(13.0, 0.25)

        bd=box2d.b2BodyDef() 
        bd.position = (4.0, 14.0)
        bd.angle = 0.25

        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.25, 1.0)

        bd=box2d.b2BodyDef() 
        bd.position = (-10.5, 11.0)

        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(13.0, 0.25)

        bd=box2d.b2BodyDef() 
        bd.position = (-4.0, 6.0)
        bd.angle = -0.25

        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.5, 0.5)
        sd.density = 25.0

        friction = [0.75, 0.5, 0.35, 0.1, 0.0]

        for i in range(5):
            bd=box2d.b2BodyDef() 
            bd.position = (-15.0 + 4.0 * i, 28.0)
            body = self.world.CreateBody(bd) 

            sd.friction = friction[i]
            body.CreateShape(sd)
            body.SetMassFromShapes()
     
if __name__=="__main__":
     main(VaryingFriction)
