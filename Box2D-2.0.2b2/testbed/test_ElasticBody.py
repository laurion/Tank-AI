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
import math

class ElasticBody (Framework):
    name="ElasticBody"
    bodies=[]
    ground=None 
    elev=None
    joint_elev=None
    _pickle_vars=['bodies', 'ground', 'elev', 'joint_elev']
    def __init__(self):
        super(ElasticBody, self).__init__()

        # Bottom static body
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 2.0)
        sd.friction = 0.1
        sd.restitution = 0.1
        bd=box2d.b2BodyDef() 
        bd.position = (-1.0, -7.5)
        self.ground = self.world.CreateBody(bd)
        self.ground.CreateShape(sd)
        # Upper static body
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(20.0, 0.50,(0,0),0.047*box2d.b2_pi)
        sd.friction    = 0.01
        sd.restitution = 0.001
        bd=box2d.b2BodyDef() 
        bd.position = (-20, 93.0)
        g = self.world.CreateBody(bd) 
        g.CreateShape(sd)
        sd.SetAsBox(15, 0.50,(-15.0,12.5),0.0)
        g.CreateShape(sd)

        sd.SetAsBox(20,0.5,(0.0,-25.0),-0.5)
        g.CreateShape(sd)
        # Left channel left wall
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.7, 55.0)
        sd.friction    = 0.1
        sd.restitution = 0.1
        bd=box2d.b2BodyDef() 
        bd.position = (-49.3, 50.0)
        g = self.world.CreateBody(bd) 
        g.CreateShape(sd)
        # Right wall
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.7, 55.0)
        sd.friction    = 0.1
        sd.restitution = 0.1
        bd=box2d.b2BodyDef() 
        bd.position = (45, 50.0)
        g = self.world.CreateBody(bd) 
        g.CreateShape(sd)
        # Left channel right upper wall
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.5, 20.0)
        sd.friction    = 0.05
        sd.restitution = 0.01
        bd=box2d.b2BodyDef() 
        bd.position = (-42.0, 70.0)
        bd.angle = -0.03*box2d.b2_pi
        g = self.world.CreateBody(bd) 
        g.CreateShape(sd)
        # Left channel right lower wall
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.50, 23.0)
        sd.friction    = 0.05
        sd.restitution = 0.01
        bd=box2d.b2BodyDef() 
        bd.position = (-44.0, 27.0)
        g = self.world.CreateBody(bd) 
        g.CreateShape(sd)
        # Bottom motors
        cd=box2d.b2CircleDef() 
        cd.radius   = 3.0
        cd.density  = 15.0
        cd.friction = 1
        cd.restitution = 0.2
        # 1.
        bd.position = (-40.0,2.5)
        body = self.world.CreateBody(bd) 
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr=box2d.b2RevoluteJointDef() 
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        jr.maxMotorTorque = 30000
        jr.enableMotor    = True
        jr.motorSpeed     = 20
        self.world.CreateJoint(jr)
        # 1. left down
        bd.position = (-46.0,-2.5)
        cd.radius = 1.5
        jr.motorSpeed  = -20
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        sd.SetAsBox(2.0, 0.50)
        body.CreateShape(sd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter())
        self.world.CreateJoint(jr)
        # 2.
        cd.radius   = 3.0
        jr.motorSpeed  = 20
        bd.position = (-32.0,2.5)
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        self.world.CreateJoint(jr)
        # 3.
        jr.motorSpeed     = 20
        bd.position = (-24.0,1.5)
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        self.world.CreateJoint(jr)
        # 4.
        bd.position = (-16.0,0.8)
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        self.world.CreateJoint(jr)
        # 5.
        bd.position = (-8.0,0.5)
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        self.world.CreateJoint(jr)
        # 6.
        bd.position = (0.0,0.1)
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        self.world.CreateJoint(jr)
        # 7.
        bd.position = (8.0,-0.5)
        body = self.world.CreateBody(bd)
        body.CreateShape(cd)
        sd.SetAsBox(3.7, 0.5)
        body.CreateShape(sd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter()+(0,1))
        self.world.CreateJoint(jr)
        # 8. right rotator
        sd.SetAsBox(5, 0.5)
        sd.density = 2.0
        bd.position = (18.0,1)
        rightmotor = self.world.CreateBody(bd) #
        rightmotor.CreateShape(sd)
        sd.SetAsBox(4.5, 0.5, (0,0),box2d.b2_pi/3)
        rightmotor.CreateShape(sd)
        sd.SetAsBox(4.5, 0.5, (0,0),box2d.b2_pi*2/3)
        rightmotor.CreateShape(sd)
        cd.radius = 4.2
        rightmotor.CreateShape(cd)
        rightmotor.SetMassFromShapes()
        jr.Initialize (g,rightmotor,rightmotor.GetWorldCenter())
        jr.maxMotorTorque = 70000
        jr.motorSpeed     = -4
        self.world.CreateJoint(jr)
        # 9. left rotator
        sd.SetAsBox(8.5, 0.5)
        sd.density = 2.0
        bd.position = (-34.0,17)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)
        sd.SetAsBox(8.5, 0.5, (0,0),box2d.b2_pi*.5)
        body.CreateShape(sd)
        cd.radius = 7
        cd.friction = 0.9
        body.CreateShape(cd)
        body.SetMassFromShapes()
        jr.Initialize (g,body,body.GetWorldCenter())
        jr.maxMotorTorque = 100000
        jr.motorSpeed     = -5
        self.world.CreateJoint(jr)
        # big compressor
        sd.SetAsBox(3.0,4)
        sd.density = 10.0
        bd.position = (-16.0,17)
        hammerleft = self.world.CreateBody(bd) 
        hammerleft.CreateShape(sd)
        hammerleft.SetMassFromShapes()
        jd=box2d.b2DistanceJointDef() 
        jd.Initialize(body, hammerleft, body.GetWorldCenter()+(0,6), hammerleft.GetWorldCenter() )
        self.world.CreateJoint(jd)

        bd.position = (4.0,17)
        hammerright = self.world.CreateBody(bd) 
        hammerright.CreateShape(sd)
        hammerright.SetMassFromShapes()
        jd.Initialize(body, hammerright, body.GetWorldCenter()-(0,6), hammerright.GetWorldCenter() )
        self.world.CreateJoint(jd)
        # pusher
        sd.SetAsBox(6,0.75)
        bd.position = (-21.0,9)
        pusher = self.world.CreateBody(bd) #
        pusher.CreateShape(sd)
        sd.SetAsBox(2,1.5,(-5,0),0)
        pusher.SetMassFromShapes()
        pusher.CreateShape(sd)
        jd.Initialize(rightmotor,pusher,rightmotor.GetWorldCenter()+(-8.0,0),
            pusher.GetWorldCenter()+(5.0,0) )
        self.world.CreateJoint(jd)
        # Static bodies above motors
        sd=box2d.b2PolygonDef() 
        cd=box2d.b2CircleDef() 
        sd.SetAsBox(9.0, 0.5)
        sd.friction    = 0.05
        sd.restitution = 0.01
        bd=box2d.b2BodyDef() 
        bd.position = (-15.5, 12)
        bd.angle = 0.0
        g = self.world.CreateBody(bd) 
        g.CreateShape(sd)

        sd.SetAsBox(8, 0.5, (23,0),0)
        g.CreateShape(sd)
        # compressor statics
        sd.SetAsBox(7.0, 0.5, (-2,9),0)
        g.CreateShape(sd)
        sd.SetAsBox(9.0, 0.5, (22,9),0)
        g.CreateShape(sd)

        sd.SetAsBox(19.0, 0.5, (-9,15),-0.05)
        g.CreateShape(sd)
        sd.SetAsBox(4.7, 0.5, (15,11.5),-0.5)
        g.CreateShape(sd)
        # below compressor
        sd.SetAsBox(26.0, 0.3, (17,-4.4),-0.02)
        g.CreateShape(sd)
        cd.radius   = 1.0	
        cd.friction = 1.0
        cd.localPosition = (29,-6)
        g.CreateShape(cd)
        cd.radius   = 0.7
        cd.localPosition = (-2,-4.5)
        g.CreateShape(cd)
        # Elevator
        bd=box2d.b2BodyDef() 
        cd=box2d.b2CircleDef() 
        sd=box2d.b2PolygonDef() 

        bd.position = (40.0,4.0)
        self.elev = self.world.CreateBody(bd)

        sd.SetAsBox(0.5, 2.5,(3.0,-3.0), 0)
        sd.density     = 1
        sd.friction    = 0.01
        self.elev.CreateShape(sd)
        sd.SetAsBox(7.0, 0.5, (-3.5,-5.5), 0)
        self.elev.CreateShape(sd)
        sd.SetAsBox(0.5, 2.5, (-11,-3.5), 0)
        self.elev.CreateShape(sd)
        self.elev.SetMassFromShapes()

        jp=box2d.b2PrismaticJointDef() 
        jp.Initialize(self.ground,self.elev, bd.position, (0.0, 1.0))
        jp.lowerTranslation =  0.0
        jp.upperTranslation = 100.0
        jp.enableLimit = True
        jp.enableMotor = True
        jp.maxMotorForce = 10000
        jp.motorSpeed    = 0
        self.joint_elev = self.world.CreateJoint(jp)

        # Korb
        sd.SetAsBox(2.3, 0.5,(1,0.0), 0.0)
        sd.density = 0.5
        bd.position = (29.0,6.5)
        body = self.world.CreateBody(bd) #
        body.CreateShape(sd)
        sd.SetAsBox(2.5, 0.5,(3.0,-2), box2d.b2_pi/2)
        body.CreateShape(sd)
        sd.SetAsBox(4.6, 0.5,(7.8,-4.0), 0)
        body.CreateShape(sd)
        sd.SetAsBox(0.5, 4.5,(12,0.0), 0)
        body.CreateShape(sd)

        sd.SetAsBox(0.5, 0.5,(13,4.0), 0)
        body.CreateShape(sd)

        cd.radius   = 0.7
        cd.density  = 1
        cd.friction = 0.01
        cd.localPosition = (0,0)
        body.CreateShape(cd)
        body.SetMassFromShapes()

        jr=box2d.b2RevoluteJointDef() 
        jr.Initialize(self.elev,body, bd.position)
        jr.enableLimit = True
        jr.lowerAngle  = -0.2
        jr.upperAngle  = box2d.b2_pi*1.1
        jr.collideConnected = True
        self.world.CreateJoint(jr)
        # upper body exit
        sd.SetAsBox(14.0, 0.5,(-3.5,-10.0), 0.0)
        bd.position = (17.5,96.0)
        body = self.world.CreateBody(bd)
        body.CreateShape(sd)

        # "Elastic body" 64 bodies - something like a lin. elastic compound
        # connected via dynamic forces (springs)
        bodies = self.bodies
        for i in range(64):
            bodies.append(None)

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(0.55, 0.55)
        sd.density    = 1.5
        sd.friction   = 0.01
        sd.filter.groupIndex = -1
        startpoint=(30,20)
        bd=box2d.b2BodyDef() 
        bd.isBullet   = False
        bd.allowSleep = False
        for i in range(8):
            for j in range(8):
                bd.position = (j*1.02, 2.51 + 1.02 * i)
                bd.position  += startpoint
                body  = self.world.CreateBody(bd)
                bodies[8*i+j] = body
                body.CreateShape(sd)
                body.SetMassFromShapes()

         #  Apply dynamic forces (springs) and check elevator state
    def Step(self, settings):
        bodies = self.bodies
        for i in range(8):
            for j in range(8):
                zero =(0.0,0.0)
                down =(0.0, -0.5)
                up   =(0.0, 0.5)
                right=(0.5, 0.0)
                left =(-0.5, 0.0)
                ind  = i*8+j
                indr = ind+1
                indd = ind+8
                spring = 500.0
                damp   = 5.0
                if j<7:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[indr]),zero,spring, damp, 1.0)
                    self.AddSpringForce((bodies[ind]),right,(bodies[indr]),left,0.5*spring, damp, 0.0)
                if i<7:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[indd]),zero,spring, damp, 1.0)
                    self.AddSpringForce((bodies[ind]),up,(bodies[indd]),down,0.5*spring,damp,0.0)
                inddr = indd + 1
                inddl = indd - 1
                drdist = math.sqrt(2.0)
                if i < 7 and j < 7:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[inddr]),zero,spring, damp, drdist)
                if i < 7 and j > 0:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[inddl]),zero,spring, damp, drdist)

                indr = ind+2
                indd = ind+8*2
                if j<6:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[indr]),zero,spring, damp, 2.0)
                if i<6:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[indd]),zero,spring,damp,2.0)

                inddr = indd + 2
                inddl = indd - 2
                drdist = math.sqrt(2.0)*2.0
                if i < 6 and j < 6:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[inddr]),zero,spring, damp, drdist)
                if i < 6 and j > 1:
                    self.AddSpringForce((bodies[ind]),zero,(bodies[inddl]),zero,spring, damp, drdist)

        # Check if bodies are near elevator
        #  Look if the body to lift is near the elevator
        p1 = bodies[0].GetWorldCenter()
        p2 = bodies[63].GetWorldCenter()
        #    self.elev:   elevator prism. joint
        e = self.elev.GetWorldCenter() + (0,7)

        # maybe not the best way to do it...
        # Bodies reached the elevator side
        if p1.x>e.x or p2.x>e.x:            # go up
            if (p1.y<e.y or p2.y<e.y ) and ( self.joint_elev.GetJointTranslation()<=self.joint_elev.GetLowerLimit()+1):
                self.joint_elev.SetMotorSpeed(20)
                #print "lift goes up trans: %G" % self.joint_elev.GetJointTranslation()

        # go down
        if self.joint_elev.GetJointTranslation() >= self.joint_elev.GetUpperLimit()-2:
            self.joint_elev.SetMotorSpeed(-15)
            #printf("lift goes down: %G\n",self.joint_elev.GetJointTranslation())

        super(ElasticBody, self).Step(settings)

    # Add a spring force
    def AddSpringForce(self, bA, localA, bB, localB, k, friction, desiredDist):
        pA = bA.GetWorldPoint(localA)
        pB = bB.GetWorldPoint(localB)
        diff=pB - pA
        #Find velocities of attach points
        vA = bA.GetLinearVelocity() - box2d.b2Cross(bA.GetWorldVector(localA), bA.GetAngularVelocity())
        vB = bB.GetLinearVelocity() - box2d.b2Cross(bB.GetWorldVector(localB), bB.GetAngularVelocity())

        vdiff=vB-vA
        dx = diff.Normalize() #normalizes diff and puts length into dx
        vrel = vdiff.x*diff.x + vdiff.y*diff.y
        forceMag = -k*(dx-desiredDist) - friction*vrel

        diff *= forceMag 
        bB.ApplyForce(diff, bA.GetWorldPoint(localA))
        diff *= -1.0
        bA.ApplyForce(diff, bB.GetWorldPoint(localB))

if __name__=="__main__":
     main(ElasticBody)
