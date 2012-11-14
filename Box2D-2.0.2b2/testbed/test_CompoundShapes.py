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
class CompoundShapes (Framework):
    name="CompoundShapes"
    def __init__(self):
        super(CompoundShapes, self).__init__()
        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)
        body = self.world.CreateBody(bd) 

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 10.0)
        body.CreateShape(sd)

        sd1=box2d.b2CircleDef() 
        sd1.radius = 0.5
        sd1.localPosition = (-0.5, 0.5)
        sd1.density = 2.0

        sd2=box2d.b2CircleDef() 
        sd2.radius = 0.5
        sd2.localPosition = (0.5, 0.5)
        sd2.density = 0.0 

        for i in range(10):
            x = box2d.b2Random(-0.1, 0.1)
            bd=box2d.b2BodyDef() 
            bd.position = (x + 5.0, 1.05 + 2.5 * i)
            bd.angle = box2d.b2Random(-box2d.b2_pi, box2d.b2_pi)
            body = self.world.CreateBody(bd) 
            body.CreateShape(sd1)
            body.CreateShape(sd2)
            body.SetMassFromShapes()

        sd1=box2d.b2PolygonDef() 
        sd1.SetAsBox(0.25, 0.5)
        sd1.density = 2.0

        sd2=box2d.b2PolygonDef() 
        sd2.SetAsBox(0.25, 0.5, (0.0, -0.5), 0.5 * box2d.b2_pi)
        sd2.density = 2.0

        for i in range(10):
            x = box2d.b2Random(-0.1, 0.1)
            bd=box2d.b2BodyDef() 
            bd.position = (x - 5.0, 1.05 + 2.5 * i)
            bd.angle = box2d.b2Random(-box2d.b2_pi, box2d.b2_pi)
            body = self.world.CreateBody(bd) 
            body.CreateShape(sd1)
            body.CreateShape(sd2)
            body.SetMassFromShapes()

        xf1 = box2d.b2XForm()
        xf1.R.Set(0.3524 * box2d.b2_pi)
        xf1.position = box2d.b2Mul(xf1.R, (1.0, 0.0))

        sd1=box2d.b2PolygonDef() 
        sd1.vertexCount = 3
        sd1.setVertex(0, box2d.b2Mul(xf1, (-1.0, 0.0)))
        sd1.setVertex(1, box2d.b2Mul(xf1, (1.0, 0.0)))
        sd1.setVertex(2, box2d.b2Mul(xf1, (0.0, 0.5)))
        sd1.density = 2.0

        xf2 = box2d.b2XForm()
        xf2.R.Set(-0.3524 * box2d.b2_pi)
        xf2.position = box2d.b2Mul(xf2.R, (-1.0, 0.0))

        sd2=box2d.b2PolygonDef() 
        sd2.vertexCount = 3
        sd2.setVertex(0, box2d.b2Mul(xf2, (-1.0, 0.0)))
        sd2.setVertex(1, box2d.b2Mul(xf2, (1.0, 0.0)))
        sd2.setVertex(2, box2d.b2Mul(xf2, (0.0, 0.5)))
        sd2.density = 2.0

        for i in range(10):
            x = box2d.b2Random(-0.1, 0.1)
            bd=box2d.b2BodyDef() 
            bd.position = (x, 2.05 + 2.5 * i)
            bd.angle = 0.0
            body = self.world.CreateBody(bd) 
            body.CreateShape(sd1)
            body.CreateShape(sd2)
            body.SetMassFromShapes()

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
        bd.position = ( 0.0, 2.0 )
        body = self.world.CreateBody(bd) 
        body.CreateShape(sd_bottom)
        body.CreateShape(sd_left)
        body.CreateShape(sd_right)
        body.SetMassFromShapes()

if __name__=="__main__":
     main(CompoundShapes)
