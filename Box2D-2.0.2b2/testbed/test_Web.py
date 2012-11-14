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

# This tests distance joints, body destruction, and joint destruction.

class Web (Framework):
    name="Web"
    bodies=[]
    joints=[]
    _pickle_vars=['bodies', 'joints']
    def __init__(self):
        super(Web, self).__init__()
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)

        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        sd=box2d.b2PolygonDef()
        sd.SetAsBox(0.5, 0.5)
        sd.density = 5.0
        sd.friction = 0.2

        bd=box2d.b2BodyDef()

        bd.position = (-5.0, 5.0)
        self.bodies.append(self.world.CreateBody(bd))
        self.bodies[0].CreateShape(sd)
        self.bodies[0].SetMassFromShapes()

        bd.position = (5.0, 5.0)
        self.bodies.append(self.world.CreateBody(bd))
        self.bodies[1].CreateShape(sd)
        self.bodies[1].SetMassFromShapes()

        bd.position = (5.0, 15.0)
        self.bodies.append(self.world.CreateBody(bd))
        self.bodies[2].CreateShape(sd)
        self.bodies[2].SetMassFromShapes()

        bd.position = (-5.0, 15.0)
        self.bodies.append(self.world.CreateBody(bd))
        self.bodies[3].CreateShape(sd)
        self.bodies[3].SetMassFromShapes()

        jd=box2d.b2DistanceJointDef()

        jd.frequencyHz = 4.0
        jd.dampingRatio = 0.5

        jd.body1 = ground
        jd.body2 = self.bodies[0]
        jd.localAnchor1 = (-10.0, 10.0)
        jd.localAnchor2 = (-0.5, -0.5)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = ground
        jd.body2 = self.bodies[1]
        jd.localAnchor1 = (10.0, 10.0)
        jd.localAnchor2 = (0.5, -0.5)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = ground
        jd.body2 = self.bodies[2]
        jd.localAnchor1 = (10.0, 30.0)
        jd.localAnchor2 = (0.5, 0.5)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = ground
        jd.body2 = self.bodies[3]
        jd.localAnchor1 = (-10.0, 30.0)
        jd.localAnchor2 = (-0.5, 0.5)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = self.bodies[0]
        jd.body2 = self.bodies[1]
        jd.localAnchor1 = (0.5, 0.0)
        jd.localAnchor2 = (-0.5, 0.0)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = self.bodies[1]
        jd.body2 = self.bodies[2]
        jd.localAnchor1 = (0.0, 0.5)
        jd.localAnchor2 = (0.0, -0.5)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = self.bodies[2]
        jd.body2 = self.bodies[3]
        jd.localAnchor1 = (-0.5, 0.0)
        jd.localAnchor2 = (0.5, 0.0)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))

        jd.body1 = self.bodies[3]
        jd.body2 = self.bodies[0]
        jd.localAnchor1 = (0.0, -0.5)
        jd.localAnchor2 = (0.0, 0.5)
        p1 = jd.body1.GetWorldPoint(jd.localAnchor1)
        p2 = jd.body2.GetWorldPoint(jd.localAnchor2)
        d = p2 - p1
        jd.length = d.Length()
        self.joints.append(self.world.CreateJoint(jd))
     
    def Keyboard(self, key):
        # Note: these functions are still causing some problems
        if key==K_b:
            for body in self.bodies:
                self.bodies.remove(body)
                self.world.DestroyBody(body)
                break

        elif key==K_j:
            for joint in self.joints:
                self.joints.remove(joint)
                self.world.DestroyJoint(joint)
                break

    def Step(self, settings):
          self.DrawStringCR("This demonstrates a soft distance joint.")
          self.DrawStringCR("Press: (b) to delete a body, (j) to delete a joint")
          super(Web, self).Step(settings)
     
    def JointDestroyed(self, joint):
        if joint in self.joints:
            print "Joint destroyed and removed from the list"
            self.joints.remove(joint)

if __name__=="__main__":
     main(Web)
