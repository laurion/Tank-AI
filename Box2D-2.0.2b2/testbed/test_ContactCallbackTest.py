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
from test_main import fwContactTypes
import math

import pprint
pp = pprint.PrettyPrinter(indent=4)
# Contributed by caspin (C++ version)

class ContactCallbackTest (Framework):
    name="ContactCallbackTest"
     
    ball=None 
    bullet=None 
    ball_shape=None 
    _pickle_vars = ['ball', 'bullet', 'ball_shape']
    def __init__(self):
        super(ContactCallbackTest, self).__init__()
    
        groundBody = self.world.GetGroundBody()

        sd=box2d.b2PolygonDef() 
        sd.friction = 0
        sd.vertices = [(10.0, 10.0), (9.0, 7.0), (10.0, 0.0)]
        sd.userData = 1
        groundBody.CreateShape(sd)

        sd.vertices = [(9.0, 7.0), (8.0, 0.0), (10.0, 0.0)]
        sd.userData = 2
        groundBody.CreateShape(sd)

        sd.vertices = [(9.0, 7.0), (8.0, 5.0), (8.0, 0.0)]
        sd.userData = 3
        groundBody.CreateShape(sd)

        sd.vertices = [(8.0, 5.0), (7.0, 4.0), (8.0, 0.0)]
        sd.userData = 4
        groundBody.CreateShape(sd)

        sd.vertices = [(7.0, 4.0), (5.0, 0.0), (8.0, 0.0)]
        sd.userData = 5
        groundBody.CreateShape(sd)

        sd.vertices = [(7.0, 4.0), (5.0, 3.0), (5.0, 0.0)]
        sd.userData = 6
        groundBody.CreateShape(sd)

        sd.vertices = [(5.0, 3.0), (2.0, 2.0), (5.0, 0.0)]
        sd.userData = 7
        groundBody.CreateShape(sd)

        sd.vertices = [(2.0, 2.0), (0.0, 0.0), (5.0, 0.0)]
        sd.userData = 8
        groundBody.CreateShape(sd)

        sd.vertices = [(2.0, 2.0), (-2.0, 2.0), (0.0, 0.0)]
        sd.userData = 9
        groundBody.CreateShape(sd)

        sd.vertices = [(-5.0, 0.0), (0.0, 0.0), (-2.0, 2.0)]
        sd.userData = 10
        groundBody.CreateShape(sd)

        sd.vertices = [(-5.0, 0.0), (-2.0, 2.0), (-5.0, 3.0)]
        sd.userData = 11
        groundBody.CreateShape(sd)

        sd.vertices = [(-5.0, 0.0), (-5.0, 3.0), (-7.0, 4.0)]
        sd.userData = 12
        groundBody.CreateShape(sd)

        sd.vertices = [(-8.0, 0.0), (-5.0, 0.0), (-7.0, 4.0)]
        sd.userData = 13
        groundBody.CreateShape(sd)

        sd.vertices = [(-8.0, 0.0), (-7.0, 4.0), (-8.0, 5.0)]
        sd.userData = 14
        groundBody.CreateShape(sd)

        sd.vertices = [(-8.0, 0.0), (-8.0, 5.0), (-9.0, 7.0)]
        sd.userData = 15
        groundBody.CreateShape(sd)

        sd.vertices = [(-10.0, 0.0), (-8.0, 0.0), (-9.0, 7.0)]
        sd.userData = 16
        groundBody.CreateShape(sd)

        sd.vertices = [(-10.0, 0.0), (-9.0, 7.0), (-10.0, 10.0)]
        sd.userData = 17
        groundBody.CreateShape(sd)

        sd.SetAsBox(.5,6,(10.5,6),0)
        groundBody.CreateShape(sd)

        sd.SetAsBox(.5,6,(-10.5,6),0)
        groundBody.CreateShape(sd)

        bd=box2d.b2BodyDef() 
        bd.position=(9.5,60)
        self.ball = self.world.CreateBody( bd ) 

        cd=box2d.b2PolygonDef() 
        cd.vertexCount = 8
        w = 1.0
        b = w / (2.0 + math.sqrt(2.0))
        s = math.sqrt(2.0) * b
        cd.vertices = [( 0.5 * s, 0.0),
                       ( 0.5 * w, b),
                       ( 0.5 * w, b + s),
                       ( 0.5 * s, w),
                       (-0.5 * s, w),
                       (-0.5 * w, b + s),
                       (-0.5 * w, b),
                       (-0.5 * s, 0.0) ]

        cd.density = 1.0
        cd.userData = 'BALL'

        self.ball_shape = self.ball.CreateShape(cd)
        self.ball.SetMassFromShapes()

    def Step(self, settings):
        strings = []
        name_dict = { fwContactTypes.contactAdded : "added",
                      fwContactTypes.contactRemoved : "removed",
                      fwContactTypes.contactPersisted : "persisted" }

        strings = ["%s: %s, %s (%s)" % (name_dict[point.state], point.shape1.userData, point.shape2.userData, point.id.key)
                     for point in self.points]

        if len(strings) > 15:
            strings = strings[:14]

        for string in strings:
            self.DrawStringCR(string)

        super(ContactCallbackTest, self).Step(settings)

if __name__=="__main__":
     main(ContactCallbackTest)
