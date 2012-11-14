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

# A tribute to Gish
# Contributed by Giorgos Giagas (giorgosg)

from test_main import *
from math import cos, sin, pi

class Tribute (Framework):
    name="Tribute to Gish"
    _pickle_vars = ['bodies', 'move', 'moving', 'jump', 'radius']
    moving = None
    jump   = False
    move   = None
    def __init__(self):
        super(Tribute, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        sd.friction = 0.2
        
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        ground      = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        sd.SetAsBox(0.5, 5.0, (-50.0, 15.0), 0.0)
        ground.CreateShape(sd)
        
        sd.SetAsBox(0.5, 5.0, (50.0, 15.0), 0.0)
        ground.CreateShape(sd)

        sd.SetAsBox(3.0,1.0)
        sd.density = 3.0
        boxd = box2d.b2BodyDef()
        for i in range(0, 16, 2):
            boxd.position = (-10.1, i)
            box = self.world.CreateBody(boxd)
            box.CreateShape(sd)
            box.SetMassFromShapes()

        sd=box2d.b2CircleDef()
        sd.density    = 5.0
        sd.friction   = 0.5
        sd.radius     = 0.4
        sd.restitution= 0.0
        
        bd=box2d.b2BodyDef()
        bd.angularDamping = 0.5
        bd.linearDamping  = 0.5
        bd.fixedRotation  = True
        bd.allowSleep     = False
        self.bodies = bodies = []
        radius = self.radius = 2
        blob_center = box2d.b2Vec2(-10, +50)

        for i in range(0, 360, 15): # 24 circles
            pos = (cos(i*2*pi/360)*radius + blob_center.x,
                   sin(i*2*pi/360)*radius + blob_center.y)
            bd.position = pos
            bodyx = self.world.CreateBody(bd)
            bodyx.CreateShape(sd)
            bodyx.SetMassFromShapes()
            bodies.append(bodyx)

        bodycount = len(self.bodies)
        pjd=box2d.b2DistanceJointDef()
        for i in range(bodycount):
            body1 = bodies[i]
            body2 = bodies[i+1] if i+1<bodycount else bodies[0]
            pjd.Initialize(body1, body2,
                           (body1.position.x, body1.position.y),
                           (body2.position.x, body2.position.y))
            pjd.dampingRatio = 10.0
            joint = self.world.CreateJoint(pjd)

        # Make the simulation a bit more stable
        self.settings.positionIterations = 20
        self.settings.velocityIterations = 20
        
        # update the pygame gui, otherwise it'll be reset
        if hasattr(self, 'gui_table'): 
            self.gui_table.updateGUI(self.settings)

    def AddSpringForce(self, bA, localA, bB, localB, k, friction, desiredDist):
        pA = bA.GetWorldPoint(localA)
        pB = bB.GetWorldPoint(localB)
        diff=pB - pA
        #Find velocities of attach points
        vA = (bA.GetLinearVelocity() -
              box2d.b2Cross(bA.GetWorldVector(localA),
                            bA.GetAngularVelocity()))
        vB = (bB.GetLinearVelocity() -
              box2d.b2Cross(bB.GetWorldVector(localB),
                            bB.GetAngularVelocity()))

        vdiff=vB-vA
        dx = diff.Normalize() #normalizes diff and puts length into dx
        vrel = vdiff.x*diff.x + vdiff.y*diff.y
        forceMag = -k*(dx-desiredDist) - friction*vrel

        diff *= forceMag 
        bB.ApplyForce(diff, bA.GetWorldPoint(localA))
        diff *= -1.0
        bA.ApplyForce(diff, bB.GetWorldPoint(localB))

    def Keyboard(self, key):
        if key==K_j:
            self.jump = True
        elif key==K_a:
            self.move = 'left'
        elif key==K_d:
            self.move = 'right'
        elif key==K_s:
            self.move = None

    def Step(self, settings):
        self.DrawStringCR("Keys: (j) jump, (a) move left, (d) move right, (s) stop")
        bodycount = len(self.bodies)
        zero = (0, 0)
        if self.jump:
            force = 10000
            self.jump = False
        else:
            force = 100
        for i in range(bodycount/2):
            self.AddSpringForce(self.bodies[i], zero,
                                self.bodies[i+bodycount/2], zero,
                                force, 5.0, self.radius*2)
        if self.move:
            top_body = [(body.position.y, body) for body in self.bodies]
            top_body.sort()
            top_body = top_body[-1][1]
            if self.move == 'left':
                force = -500
            elif self.move == 'right':
                force = 500
            top_body.ApplyForce((force,0), top_body.position)
            
        super(Tribute, self).Step(settings)
    
if __name__=="__main__":
    main(Tribute)
