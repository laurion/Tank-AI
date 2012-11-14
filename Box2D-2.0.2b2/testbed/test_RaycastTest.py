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

class RaycastTest (Framework):
    name="RaycastTest"
    laserBody=None
    _pickle_vars=['laserBody']
    def __init__(self):
        super(RaycastTest, self).__init__()
        #self.world.SetGravity( (0,0) )
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        ground.CreateShape(sd)
    
        bd=box2d.b2BodyDef()
        bd.position = (0.0, 1.0)
        self.laserBody = self.world.CreateBody(bd)
        
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(5.0, 1.0)
        sd.density = 4.0
        self.laserBody.CreateShape(sd)
        self.laserBody.SetMassFromShapes()
        
        #Create a few shapes
        bd.position = (-5.0, 10.0)
        body = self.world.CreateBody(bd)
        
        cd=box2d.b2CircleDef()
        cd.radius = 3
        body.CreateShape(cd)
        
        bd.position = (5.0, 10.0)
        body = self.world.CreateBody(bd)
        
        body.CreateShape(cd)
    
    def Keyboard(self, key):
        pass

    def Step(self, settings):
        super(RaycastTest, self).Step(settings)

        if not self.laserBody:
            return

        segmentLength = 30.0
        
        segment=box2d.b2Segment()
        laserStart=(5.0-0.1,0.0)
        laserDir=(segmentLength,0.0)
        segment.p1 = self.laserBody.GetWorldPoint(laserStart)
        segment.p2 = self.laserBody.GetWorldVector(laserDir)
        segment.p2+=segment.p1
        
        for rebounds in xrange(10):
            lambda_, normal, shape = self.world.RaycastOne(segment,False,None)
            
            laserColor=box2d.b2Color(1.0,0,0)
            
            if shape:
                self.debugDraw.DrawSegment(segment.p1,(1-lambda_)*segment.p1+lambda_*segment.p2,laserColor)
            else:
                self.debugDraw.DrawSegment(segment.p1,segment.p2,laserColor)
                break

            #Bounce
            segmentLength *= (1-lambda_)

            if segmentLength <= box2d.B2_FLT_EPSILON:
                break

            laserStart = (1-lambda_)*segment.p1+lambda_*segment.p2
            laserDir = segment.p2-segment.p1
            laserDir.Normalize()
            laserDir = laserDir -2 * box2d.b2Dot(laserDir,normal) * normal
            segment.p1 = laserStart-0.1*laserDir
            segment.p2 = laserStart+segmentLength*laserDir


if __name__=="__main__":
    main(RaycastTest)
