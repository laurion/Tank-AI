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

class SensorTest (Framework):
    name="SensorTest"
    sensor=None
    _pickle_vars=['sensor']
    def __init__(self):
        super(SensorTest, self).__init__()
        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)

        ground = self.world.CreateBody(bd) 

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 10.0)
        ground.CreateShape(sd)

        if True:
            sd=box2d.b2PolygonDef() 
            sd.SetAsBox(10.0, 2.0, (0.0, 20.0), 0.0)
            sd.isSensor = True
            self.sensor = ground.CreateShape(sd)

        else:
            # Alternative test:
            cd=box2d.b2CircleDef() 
            cd.isSensor = True
            cd.radius = 5.0
            cd.localPosition = (0.0, 20.0)
            self.sensor = ground.CreateShape(cd)

        sd=box2d.b2CircleDef() 
        sd.radius = 1.0
        sd.density = 1.0

        for i in range(10):
            bd=box2d.b2BodyDef() 
            bd.position = (0.0 + 3.0 * i, 20.0)
            
            body = self.world.CreateBody(bd) 

            body.CreateShape(sd)
            body.SetMassFromShapes()

    def Step(self, settings):
        # Traverse the contact results. Apply a force on shapes
        # that overlap the sensor.
        for point in self.points:
            if point.state == fwContactTypes.contactPersisted:
                continue
            
            shape1, shape2=point.shape1, point.shape2
            other=None
            
            if shape1 == self.sensor:
                other = shape2.GetBody()
            elif shape2 == self.sensor:
                other = shape1.GetBody()
            else:
                continue

            ground = self.sensor.GetBody()
            typedshape = self.sensor
            if self.sensor.GetType() == box2d.e_circleShape:
                center = ground.GetWorldPoint(typedshape.GetLocalPosition())
            elif self.sensor.GetType() == box2d.e_polygonShape:
                center = ground.GetWorldPoint(typedshape.GetCentroid())
            else:
                print "don't know how to get the center of this shape, using (0,0)"
                center = ground.GetWorldPoint(box2d.b2Vec2(0.0, 0.0))

            d = center - point.position

            if d.LengthSquared() < box2d.B2_FLT_EPSILON * box2d.B2_FLT_EPSILON:
                continue

            d.Normalize()
            F = 100.0 * d
            other.ApplyForce(F, point.position)

        super(SensorTest, self).Step(settings)


if __name__=="__main__":
     main(SensorTest)
