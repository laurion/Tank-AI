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

class Dominos (Framework):
    name="Dominos"     
    def __init__(self):
        super(Dominos, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)

        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        b1 = self.world.CreateBody(bd)
        b1.CreateShape(sd)

        sd=box2d.b2PolygonDef()
        sd.SetAsBox(6.0, 0.25)

        bd=box2d.b2BodyDef()
        bd.position = (-1.5, 10.0)
        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.1, 1.0)
        sd.density = 20.0
        sd.friction = 0.1

        
        for i in range(10):
            bd=box2d.b2BodyDef() 
            bd.position = (-6.0 + 1.0 * i, 11.25)
            body = self.world.CreateBody(bd) 
            body.CreateShape(sd)
            body.SetMassFromShapes()

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(7.0, 0.25, (0,0), 0.3)

        bd=box2d.b2BodyDef() 
        bd.position = (1.0, 6.0)
        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.25, 1.5)

        bd=box2d.b2BodyDef() 
        bd.position = (-7.0, 4.0)
        b2 = self.world.CreateBody(bd)
        b2.CreateShape(sd)

        
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(6.0, 0.125)
        sd.density = 10.0

        bd=box2d.b2BodyDef() 
        bd.position = (-0.9, 1.0)
        bd.angle = -0.15

        b3 = self.world.CreateBody(bd)
        b3.CreateShape(sd)
        b3.SetMassFromShapes()

        jd=box2d.b2RevoluteJointDef() 

        anchor=(-2.0, 1.0) 
        jd.Initialize(b1, b3, anchor)
        jd.collideConnected = True
        self.world.CreateJoint(jd)

        
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.25, 0.25)
        sd.density = 10.0

        bd=box2d.b2BodyDef() 
        bd.position = (-10.0, 15.0)
        b4 = self.world.CreateBody(bd)
        b4.CreateShape(sd)
        b4.SetMassFromShapes()

        anchor = (-7.0, 15.0)
        jd.Initialize(b2, b4, anchor)
        self.world.CreateJoint(jd)

        
        bd=box2d.b2BodyDef() 
        bd.position = (6.5, 3.0)
        b5 = self.world.CreateBody(bd)

        sd=box2d.b2PolygonDef() 
        sd.density = 10.0
        sd.friction = 0.1

        sd.SetAsBox(1.0, 0.1, (0.0, -0.9), 0.0)
        b5.CreateShape(sd)

        sd.SetAsBox(0.1, 1.0, (-0.9, 0.0), 0.0)
        b5.CreateShape(sd)

        sd.SetAsBox(0.1, 1.0, (0.9, 0.0), 0.0)
        b5.CreateShape(sd)

        b5.SetMassFromShapes()

        anchor = (6.0, 2.0)
        jd.Initialize(b1, b5, anchor)
        self.world.CreateJoint(jd)

        
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(1.0, 0.1)
        sd.density = 30.0
        sd.friction = 0.2

        bd=box2d.b2BodyDef() 
        bd.position = (6.5, 4.1)
        b6 = self.world.CreateBody(bd)
        b6.CreateShape(sd)
        b6.SetMassFromShapes()

        anchor = (7.5, 4.0)
        jd.Initialize(b5, b6, anchor)
        self.world.CreateJoint(jd)

        
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.1, 1.0)
        sd.density = 10.0

        bd=box2d.b2BodyDef() 
        bd.position = (7.4, 1.0)

        b7 = self.world.CreateBody(bd)
        b7.CreateShape(sd)
        b7.SetMassFromShapes()

        djd=box2d.b2DistanceJointDef() 
        djd.body1 = b3
        djd.body2 = b7
        djd.localAnchor1 = (6.0, 0.0)
        djd.localAnchor2 = (0.0, -1.0)
        d = djd.body2.GetWorldPoint(djd.localAnchor2) - djd.body1.GetWorldPoint(djd.localAnchor1)
        djd.length = d.Length()
        self.world.CreateJoint(djd)

        sd=box2d.b2CircleDef() 
        sd.radius = 0.2
        sd.density = 10.0

        
        for i in range(4):
            bd=box2d.b2BodyDef() 
            bd.position = (5.9 + 2.0 * sd.radius * i, 2.4)
            body = self.world.CreateBody(bd) 
            body.CreateShape(sd)
            body.SetMassFromShapes()

if __name__=="__main__":
     main(Dominos)
