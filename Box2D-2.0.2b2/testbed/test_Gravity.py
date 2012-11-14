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
class Gravity (Framework):
    name="Gravity"
    gc=None
    bodies = [ ]
    _pickle_vars=['bodies', 'gc']
    def __init__(self):
        super(Gravity, self).__init__()
        gcd = box2d.b2GravityControllerDef() 
        gcd.G=0.8
        gcd.invSqr=True
        self.gc = self.world.CreateController(gcd)
        
        self.world.gravity = (0,0)

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, 20.0)
        ground = self.world.CreateBody(bd) 

        sd=box2d.b2PolygonDef() 
        sd.density = 0.0
        sd.restitution = 1

        sd.SetAsBox(0.2, 20.0, (-20.0, 0.0), 0.0)
        ground.CreateShape(sd)

        sd.SetAsBox(0.2, 20.0, (20.0, 0.0), 0.0)
        ground.CreateShape(sd)

        sd.SetAsBox(0.2, 20.0, (0.0, -20.0), 0.5 * box2d.b2_pi)
        ground.CreateShape(sd)

        sd.SetAsBox(0.2, 20.0, (0.0, 20.0), -0.5 * box2d.b2_pi)
        ground.CreateShape(sd)
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        
        sd=box2d.b2CircleDef()
        sd.density = 1.0
        sd.isBullet=True
        
        bd=box2d.b2BodyDef()

        for i in range(1,4):
            sd.radius = 0.25*i
            bd.position = (0.25*i, 2.0 + 7.5 * i)
            
            body=self.world.CreateBody(bd)
            self.bodies.append(body)
            body.CreateShape(sd)
            body.SetMassFromShapes()
            self.gc.AddBody(body)

            body.ApplyForce( (-10*i, 2), body.GetWorldCenter() )

if __name__=="__main__":
    main(Gravity)
