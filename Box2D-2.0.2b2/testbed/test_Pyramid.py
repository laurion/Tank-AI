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
class Pyramid (Framework):
    name="Pyramid"
    def __init__(self):
        super(Pyramid, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef() 
        a = 0.5
        sd.SetAsBox(a, a)
        sd.density = 5.0

        x=box2d.b2Vec2(-10.0, 0.75)
        deltaX=(0.5625, 2.0)
        deltaY=(1.125, 0.0)

        for i in range(25):
            y = x.copy()

            for j in range(i, 25):
                bd=box2d.b2BodyDef() 
                bd.position = y
                body = self.world.CreateBody(bd) 
                body.CreateShape(sd)
                body.SetMassFromShapes()
                y += deltaY

            x += deltaX

if __name__=="__main__":
     main(Pyramid)
