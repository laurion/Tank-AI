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

class ShapeEditing (Framework):
    name="ShapeEditing"
    body=None
    shape1=None
    shape2=None
    _pickle_vars=['body', 'shape1', 'shape2']
    def __init__(self):
        super(ShapeEditing, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)

        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)

        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        bodydef=box2d.b2BodyDef()
        bodydef.position = (0.0, 10.0)
        self.body = self.world.CreateBody(bodydef)

        sd=box2d.b2PolygonDef()
        sd.SetAsBox(4.0, 4.0, (0.0, 0.0), 0.0)
        sd.density = 10.0
        self.shape1 = self.body.CreateShape(sd)
        self.body.SetMassFromShapes()

        self.shape2 = None
             
    def Keyboard(self, key):
        if not self.body:
            return

        if key==K_c:
            if not self.shape2:
                sd=box2d.b2CircleDef()
                sd.radius = 3.0
                sd.density = 10.0
                sd.localPosition = (0.5, -4.0)
                self.shape2 = self.body.CreateShape(sd)
                self.body.SetMassFromShapes()
                self.body.WakeUp()

        elif key==K_d:
            if self.shape2:
                self.body.DestroyShape(self.shape2)
                self.shape2 = None
                self.body.SetMassFromShapes()
                self.body.WakeUp()

    def Step(self, settings):
        self.DrawStringCR("Press: (c) create a shape, (d) destroy the shape")
        super(ShapeEditing, self).Step(settings)

if __name__=="__main__":
     main(ShapeEditing)
