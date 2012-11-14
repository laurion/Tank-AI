#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/python
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

global k_scale
k_scale = 3.0

class Test_Biped(Framework):
    name = "Biped"
    biped = None
    _pickle_vars=[]

    def __init__(self):
        super(Test_Biped, self).__init__()
        k_restitution = 1.4

        bd=box2d.b2BodyDef()
        bd.position = (0.0, 20.0)
        body = self.world.CreateBody(bd)

        sd=box2d.b2PolygonDef()
        sd.density = 0.0
        sd.restitution = k_restitution

        sd.SetAsBox(0.1, 10.0, (-10.0, 0.0), 0.0)
        body.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (10.0, 0.0), 0.0)
        body.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (0.0, -10.0), 0.5 * box2d.b2_pi)
        body.CreateShape(sd)

        sd.SetAsBox(0.1, 10.0, (0.0, 10.0), -0.5 * box2d.b2_pi)
        body.CreateShape(sd)

        self.biped = Biped(self.world, box2d.b2Vec2(0.0, 20.0))

        for i in range(8):
            bd=box2d.b2BodyDef()
            bd.position = (5.0, 20.0 + i)
            bd.isBullet = True
            body = self.world.CreateBody(bd) 
            body.SetLinearVelocity(box2d.b2Vec2(0.0, -100.0))
            body.SetAngularVelocity(box2d.b2Random(-50.0, 50.0))

            sd=box2d.b2CircleDef()
            sd.radius = 0.25
            sd.density = 15.0
            sd.restitution = k_restitution
            body.CreateShape(sd)
            body.SetMassFromShapes()

class Biped(object):
    world = None
    LFoot=None
    RFoot=None
    LCalf=None
    RCalf=None
    LThigh=None
    RThigh=None
    
    Pelvis=None
    Stomach=None
    Chest=None
    Neck=None
    Head=None
    LUpperArm=None
    RUpperArm=None
    LForearm=None
    RForearm=None
    LHand=None
    RHand=None

    LAnkle=None
    RAnkle=None
    LKnee=None
    RKnee=None
    LHip=None
    RHip=None
    
    LowerAbs=None
    UpperAbs=None
    LowerNeck=None
    UpperNeck=None
    LShoulder=None
    RShoulder=None
    LElbow=None
    RElbow=None
    LWrist=None
    RWrist=None

    def __init__(self, world, position):
        self.world = world
        bdef = BipedDef()
        bd=box2d.b2BodyDef()

        # create body parts
        bd = bdef.LFootDef
        bd.position += position
        self.LFoot = self.world.CreateBody(bd)
        self.LFoot.CreateShape(bdef.LFootPoly)
        self.LFoot.SetMassFromShapes()

        bd = bdef.RFootDef
        bd.position += position
        self.RFoot = self.world.CreateBody(bd)
        self.RFoot.CreateShape(bdef.RFootPoly)
        self.RFoot.SetMassFromShapes()

        bd = bdef.LCalfDef
        bd.position += position
        self.LCalf = self.world.CreateBody(bd)
        self.LCalf.CreateShape(bdef.LCalfPoly)
        self.LCalf.SetMassFromShapes()

        bd = bdef.RCalfDef
        bd.position += position
        self.RCalf = self.world.CreateBody(bd)
        self.RCalf.CreateShape(bdef.RCalfPoly)
        self.RCalf.SetMassFromShapes()

        bd = bdef.LThighDef
        bd.position += position
        self.LThigh = self.world.CreateBody(bd)
        self.LThigh.CreateShape(bdef.LThighPoly)
        self.LThigh.SetMassFromShapes()

        bd = bdef.RThighDef
        bd.position += position
        self.RThigh = self.world.CreateBody(bd)
        self.RThigh.CreateShape(bdef.RThighPoly)
        self.RThigh.SetMassFromShapes()

        bd = bdef.PelvisDef
        bd.position += position
        self.Pelvis = self.world.CreateBody(bd)
        self.Pelvis.CreateShape(bdef.PelvisPoly)
        self.Pelvis.SetMassFromShapes()

        bd = bdef.StomachDef
        bd.position += position
        self.Stomach = self.world.CreateBody(bd)
        self.Stomach.CreateShape(bdef.StomachPoly)
        self.Stomach.SetMassFromShapes()

        bd = bdef.ChestDef
        bd.position += position
        self.Chest = self.world.CreateBody(bd)
        self.Chest.CreateShape(bdef.ChestPoly)
        self.Chest.SetMassFromShapes()

        bd = bdef.NeckDef
        bd.position += position
        self.Neck = self.world.CreateBody(bd)
        self.Neck.CreateShape(bdef.NeckPoly)
        self.Neck.SetMassFromShapes()

        bd = bdef.HeadDef
        bd.position += position
        self.Head = self.world.CreateBody(bd)
        self.Head.CreateShape(bdef.HeadCirc)
        self.Head.SetMassFromShapes()

        bd = bdef.LUpperArmDef
        bd.position += position
        self.LUpperArm = self.world.CreateBody(bd)
        self.LUpperArm.CreateShape(bdef.LUpperArmPoly)
        self.LUpperArm.SetMassFromShapes()

        bd = bdef.RUpperArmDef
        bd.position += position
        self.RUpperArm = self.world.CreateBody(bd)
        self.RUpperArm.CreateShape(bdef.RUpperArmPoly)
        self.RUpperArm.SetMassFromShapes()

        bd = bdef.LForearmDef
        bd.position += position
        self.LForearm = self.world.CreateBody(bd)
        self.LForearm.CreateShape(bdef.LForearmPoly)
        self.LForearm.SetMassFromShapes()

        bd = bdef.RForearmDef
        bd.position += position
        self.RForearm = self.world.CreateBody(bd)
        self.RForearm.CreateShape(bdef.RForearmPoly)
        self.RForearm.SetMassFromShapes()

        bd = bdef.LHandDef
        bd.position += position
        self.LHand = self.world.CreateBody(bd)
        self.LHand.CreateShape(bdef.LHandPoly)
        self.LHand.SetMassFromShapes()

        bd = bdef.RHandDef
        bd.position += position
        self.RHand = self.world.CreateBody(bd)
        self.RHand.CreateShape(bdef.RHandPoly)
        self.RHand.SetMassFromShapes()

        # link body parts
        bdef.LAnkleDef.body1	= self.LFoot
        bdef.LAnkleDef.body2	= self.LCalf
        bdef.RAnkleDef.body1	= self.RFoot
        bdef.RAnkleDef.body2	= self.RCalf
        bdef.LKneeDef.body1		= self.LCalf
        bdef.LKneeDef.body2		= self.LThigh
        bdef.RKneeDef.body1		= self.RCalf
        bdef.RKneeDef.body2		= self.RThigh
        bdef.LHipDef.body1		= self.LThigh
        bdef.LHipDef.body2		= self.Pelvis
        bdef.RHipDef.body1		= self.RThigh
        bdef.RHipDef.body2		= self.Pelvis
        bdef.LowerAbsDef.body1	= self.Pelvis
        bdef.LowerAbsDef.body2	= self.Stomach
        bdef.UpperAbsDef.body1	= self.Stomach
        bdef.UpperAbsDef.body2	= self.Chest
        bdef.LowerNeckDef.body1	= self.Chest
        bdef.LowerNeckDef.body2	= self.Neck
        bdef.UpperNeckDef.body1	= self.Chest
        bdef.UpperNeckDef.body2	= self.Head
        bdef.LShoulderDef.body1	= self.Chest
        bdef.LShoulderDef.body2	= self.LUpperArm
        bdef.RShoulderDef.body1	= self.Chest
        bdef.RShoulderDef.body2	= self.RUpperArm
        bdef.LElbowDef.body1	= self.LForearm
        bdef.LElbowDef.body2	= self.LUpperArm
        bdef.RElbowDef.body1	= self.RForearm
        bdef.RElbowDef.body2	= self.RUpperArm
        bdef.LWristDef.body1	= self.LHand
        bdef.LWristDef.body2	= self.LForearm
        bdef.RWristDef.body1	= self.RHand
        bdef.RWristDef.body2	= self.RForearm

        # create joints
        self.LAnkle		= self.world.CreateJoint(bdef.LAnkleDef)
        self.RAnkle		= self.world.CreateJoint(bdef.RAnkleDef)
        self.LKnee		= self.world.CreateJoint(bdef.LKneeDef)
        self.RKnee		= self.world.CreateJoint(bdef.RKneeDef)
        self.LHip		= self.world.CreateJoint(bdef.LHipDef)
        self.RHip		= self.world.CreateJoint(bdef.RHipDef)
        self.LowerAbs	= self.world.CreateJoint(bdef.LowerAbsDef)
        self.UpperAbs	= self.world.CreateJoint(bdef.UpperAbsDef)
        self.LowerNeck	= self.world.CreateJoint(bdef.LowerNeckDef)
        self.UpperNeck	= self.world.CreateJoint(bdef.UpperNeckDef)
        self.LShoulder	= self.world.CreateJoint(bdef.LShoulderDef)
        self.RShoulder	= self.world.CreateJoint(bdef.RShoulderDef)
        self.LElbow		= self.world.CreateJoint(bdef.LElbowDef)
        self.RElbow		= self.world.CreateJoint(bdef.RElbowDef)
        self.LWrist		= self.world.CreateJoint(bdef.LWristDef)
        self.RWrist		= self.world.CreateJoint(bdef.RWristDef)

class BipedDef(object):
    # BodyDefs
    LFootDef=box2d.b2BodyDef()
    RFootDef=box2d.b2BodyDef()
    LCalfDef=box2d.b2BodyDef()
    RCalfDef=box2d.b2BodyDef()
    LThighDef=box2d.b2BodyDef()
    RThighDef=box2d.b2BodyDef()
    
    PelvisDef=box2d.b2BodyDef()
    StomachDef=box2d.b2BodyDef()
    ChestDef=box2d.b2BodyDef()
    NeckDef=box2d.b2BodyDef()
    HeadDef=box2d.b2BodyDef()
    
    LUpperArmDef=box2d.b2BodyDef()
    RUpperArmDef=box2d.b2BodyDef()
    LForearmDef=box2d.b2BodyDef()
    RForearmDef=box2d.b2BodyDef()
    LHandDef=box2d.b2BodyDef()
    RHandDef=box2d.b2BodyDef()
   
	# Polygons
    LFootPoly=box2d.b2PolygonDef()
    RFootPoly=box2d.b2PolygonDef()
    LCalfPoly=box2d.b2PolygonDef()
    RCalfPoly=box2d.b2PolygonDef()
    LThighPoly=box2d.b2PolygonDef()
    RThighPoly=box2d.b2PolygonDef()
   
    PelvisPoly=box2d.b2PolygonDef()
    StomachPoly=box2d.b2PolygonDef()
    ChestPoly=box2d.b2PolygonDef()
    NeckPoly=box2d.b2PolygonDef()
   
    LUpperArmPoly=box2d.b2PolygonDef()
    RUpperArmPoly=box2d.b2PolygonDef()
    LForearmPoly=box2d.b2PolygonDef()
    RForearmPoly=box2d.b2PolygonDef()
    LHandPoly=box2d.b2PolygonDef()
    RHandPoly=box2d.b2PolygonDef()
   
    # Circles
    HeadCirc=box2d.b2CircleDef()
	
    # Revolute Joints
    LAnkleDef=box2d.b2RevoluteJointDef()
    RAnkleDef=box2d.b2RevoluteJointDef()
    LKneeDef=box2d.b2RevoluteJointDef()
    RKneeDef=box2d.b2RevoluteJointDef()
    LHipDef=box2d.b2RevoluteJointDef()
    RHipDef=box2d.b2RevoluteJointDef()
    
    LowerAbsDef=box2d.b2RevoluteJointDef()
    UpperAbsDef=box2d.b2RevoluteJointDef()
    LowerNeckDef=box2d.b2RevoluteJointDef()
    UpperNeckDef=box2d.b2RevoluteJointDef()
   
    LShoulderDef=box2d.b2RevoluteJointDef()
    RShoulderDef=box2d.b2RevoluteJointDef()
    LElbowDef=box2d.b2RevoluteJointDef()
    RElbowDef=box2d.b2RevoluteJointDef()
    LWristDef=box2d.b2RevoluteJointDef()
    RWristDef=box2d.b2RevoluteJointDef()
    
    count = 0   
    def __init__(self):
        # So much cleaner in Python than C++ :)
        self.iter_polys = ( self.LFootPoly, self.RFootPoly, self.LCalfPoly, self.RCalfPoly, self.LThighPoly, self.RThighPoly,
                self.PelvisPoly, self.StomachPoly, self.ChestPoly, self.NeckPoly,
                self.LUpperArmPoly, self.RUpperArmPoly, self.LForearmPoly, self.RForearmPoly, self.LHandPoly, self.RHandPoly ,
                self.HeadCirc )
        self.iter_defs = ( self.LFootDef, self.RFootDef, self.LCalfDef, self.RCalfDef, self.LThighDef, self.RThighDef,
                self.PelvisDef, self.StomachDef, self.ChestDef, self.NeckDef, self.HeadDef,
                self.LUpperArmDef, self.RUpperArmDef, self.LForearmDef, self.RForearmDef, self.LHandDef, self.RHandDef )
        self.iter_joints=( self.LAnkleDef, self.RAnkleDef, self.LKneeDef, self.RKneeDef, self.LHipDef, self.RHipDef,
                self.LowerAbsDef, self.UpperAbsDef, self.LowerNeckDef, self.UpperNeckDef,
                self.LShoulderDef, self.RShoulderDef, self.LElbowDef, self.RElbowDef, self.LWristDef, self.RWristDef )

        self.SetMotorTorque(2.0)
        self.SetMotorSpeed(0.0)
        self.SetDensity(20.0)
        self.SetRestitution(0.0)
        self.SetLinearDamping(0.0)
        self.SetAngularDamping(0.005)
        self.count -= 1
        self.SetGroupIndex(self.count)
        self.EnableMotor()
        self.EnableLimit()

        self.DefaultVertices()
        self.DefaultPositions()
        self.DefaultJoints()

        self.LFootPoly.friction = self.RFootPoly.friction = 0.85

    def IsFast(self, b):
        pass

    def SetGroupIndex(self, i):
        for o in self.iter_polys:
            o.filter.groupIndex	= i

    def SetLinearDamping(self, f):
        for d in self.iter_defs:
            d.linearDamping = f

    def SetAngularDamping(self, f):
        for d in self.iter_defs:
            d.angularDamping = f

    def SetMotorTorque(self, f):
        for j in self.iter_joints:
            j.maxMotorTorque = f

    def SetMotorSpeed(self,  f):
        for j in self.iter_joints:
            j.motorSpeed = f

    def SetDensity(self,  f):
        for o in self.iter_polys:
            o.density = f

    def SetRestitution(self, f):
        for o in self.iter_polys:
            o.restitution = f

    def EnableLimit(self):
        self.SetLimit(True)

    def DisableLimit(self):
        self.SetLimit(False)

    def SetLimit(self, b):
        for j in self.iter_joints:
            j.enableLimit = b

    def EnableMotor(self):
        self.SetMotor(True)

    def DisableMotor(self):
        self.SetMotor(False)

    def SetMotor(self, b):
        for j in self.iter_joints:
            j.enableMotor = b

    def DefaultVertices(self):
        global k_scale
        # feet
        for poly in (self.LFootPoly, self.RFootPoly):
            poly.setVertices([
                k_scale * box2d.b2Vec2(.033,.143),
                k_scale * box2d.b2Vec2(.023,.033),
                k_scale * box2d.b2Vec2(.267,.035),
                k_scale * box2d.b2Vec2(.265,.065),
                k_scale * box2d.b2Vec2(.117,.143)])
        # calves
        for poly in (self.LCalfPoly, self.RCalfPoly):
            poly.setVertices([
                k_scale * box2d.b2Vec2(.089,.016),
                k_scale * box2d.b2Vec2(.178,.016),
                k_scale * box2d.b2Vec2(.205,.417),
                k_scale * box2d.b2Vec2(.095,.417)])
        # thighs
        for poly in (self.LThighPoly, self.RThighPoly):
            poly.setVertices([
                k_scale * box2d.b2Vec2(.137,.032),
                k_scale * box2d.b2Vec2(.243,.032),
                k_scale * box2d.b2Vec2(.318,.343),
                k_scale * box2d.b2Vec2(.142,.343)])
        # pelvis
        self.PelvisPoly.setVertices([
            k_scale * box2d.b2Vec2(.105,.051),
            k_scale * box2d.b2Vec2(.277,.053),
            k_scale * box2d.b2Vec2(.320,.233),
            k_scale * box2d.b2Vec2(.112,.233),
            k_scale * box2d.b2Vec2(.067,.152)])
        # stomach
        self.StomachPoly.setVertices([
            k_scale * box2d.b2Vec2(.088,.043),
            k_scale * box2d.b2Vec2(.284,.043),
            k_scale * box2d.b2Vec2(.295,.231),
            k_scale * box2d.b2Vec2(.100,.231)])
        # chest
        self.ChestPoly.setVertices([
            k_scale * box2d.b2Vec2(.091,.042),
            k_scale * box2d.b2Vec2(.283,.042),
            k_scale * box2d.b2Vec2(.177,.289),
            k_scale * box2d.b2Vec2(.065,.289)])
        # head
        self.HeadCirc.radius = k_scale * .115
        # neck
        self.NeckPoly.setVertices([
            k_scale * box2d.b2Vec2(.038,.054),
            k_scale * box2d.b2Vec2(.149,.054),
            k_scale * box2d.b2Vec2(.154,.102),
            k_scale * box2d.b2Vec2(.054,.113)])
        # upper arms
        for poly in (self.LUpperArmPoly, self.RUpperArmPoly):
            poly.setVertices([
                k_scale * box2d.b2Vec2(.092,.059),
                k_scale * box2d.b2Vec2(.159,.059),
                k_scale * box2d.b2Vec2(.169,.335),
                k_scale * box2d.b2Vec2(.078,.335),
                k_scale * box2d.b2Vec2(.064,.248)])
        # forearms
        for poly in (self.LForearmPoly, self.RForearmPoly):
            poly.setVertices([
                k_scale * box2d.b2Vec2(.082,.054),
                k_scale * box2d.b2Vec2(.138,.054),
                k_scale * box2d.b2Vec2(.149,.296),
                k_scale * box2d.b2Vec2(.088,.296)])
        # hands
        for poly in (self.LHandPoly, self.RHandPoly):
            poly.setVertices([
                k_scale * box2d.b2Vec2(.066,.031),
                k_scale * box2d.b2Vec2(.123,.020),
                k_scale * box2d.b2Vec2(.160,.127),
                k_scale * box2d.b2Vec2(.127,.178),
                k_scale * box2d.b2Vec2(.074,.178)])

    def DefaultJoints(self):
        global k_scale
        #b.LAnkleDef.body1		= LFoot
        #b.LAnkleDef.body2		= LCalf
        #b.self.RAnkleDef.body1		= self.RFoot
        #b.self.RAnkleDef.body2		= self.RCalf
        # ankles
        anchor = k_scale * box2d.b2Vec2(-.045,-.75)
        self.LAnkleDef.localAnchor1		= self.RAnkleDef.localAnchor1	= anchor - self.LFootDef.position
        self.LAnkleDef.localAnchor2		= self.RAnkleDef.localAnchor2	= anchor - self.LCalfDef.position
        self.LAnkleDef.referenceAngle	= self.RAnkleDef.referenceAngle	= 0.0
        self.LAnkleDef.lowerAngle		= self.RAnkleDef.lowerAngle		= -0.523598776
        self.LAnkleDef.upperAngle		= self.RAnkleDef.upperAngle		= 0.523598776

        #b.self.LKneeDef.body1		= self.LCalf
        #b.self.LKneeDef.body2		= self.LThigh
        #b.self.RKneeDef.body1		= self.RCalf
        #b.self.RKneeDef.body2		= self.RThigh
        # knees
        anchor = k_scale * box2d.b2Vec2(-.030,-.355)
        self.LKneeDef.localAnchor1	= self.RKneeDef.localAnchor1		= anchor - self.LCalfDef.position
        self.LKneeDef.localAnchor2	= self.RKneeDef.localAnchor2		= anchor - self.LThighDef.position
        self.LKneeDef.referenceAngle	= self.RKneeDef.referenceAngle	= 0.0
        self.LKneeDef.lowerAngle		= self.RKneeDef.lowerAngle		= 0
        self.LKneeDef.upperAngle		= self.RKneeDef.upperAngle		= 2.61799388

        #b.self.LHipDef.body1			= self.LThigh
        #b.self.LHipDef.body2			= Pelvis
        #b.self.RHipDef.body1			= self.RThigh
        #b.self.RHipDef.body2			= Pelvis
        # hips
        anchor = k_scale * box2d.b2Vec2(.005,-.045)
        self.LHipDef.localAnchor1	= self.RHipDef.localAnchor1		= anchor - self.LThighDef.position
        self.LHipDef.localAnchor2	= self.RHipDef.localAnchor2		= anchor - self.PelvisDef.position
        self.LHipDef.referenceAngle	= self.RHipDef.referenceAngle	= 0.0
        self.LHipDef.lowerAngle		= self.RHipDef.lowerAngle		= -2.26892803
        self.LHipDef.upperAngle		= self.RHipDef.upperAngle		= 0

        #b.self.LowerAbsDef.body1		= Pelvis
        #b.self.LowerAbsDef.body2		= Stomach
        # lower abs
        anchor = k_scale * box2d.b2Vec2(.035,.135)
        self.LowerAbsDef.localAnchor1	= anchor - self.PelvisDef.position
        self.LowerAbsDef.localAnchor2	= anchor - self.StomachDef.position
        self.LowerAbsDef.referenceAngle	= 0.0
        self.LowerAbsDef.lowerAngle		= -0.523598776
        self.LowerAbsDef.upperAngle		= 0.523598776

        #b.UpperAbsDef.body1		= Stomach
        #b.UpperAbsDef.body2		= Chest
        # upper abs
        anchor = k_scale * box2d.b2Vec2(.045,.320)
        self.UpperAbsDef.localAnchor1	= anchor - self.StomachDef.position
        self.UpperAbsDef.localAnchor2	= anchor - self.ChestDef.position
        self.UpperAbsDef.referenceAngle	= 0.0
        self.UpperAbsDef.lowerAngle		= -0.523598776
        self.UpperAbsDef.upperAngle		= 0.174532925

        #b.self.LowerNeckDef.body1	= Chest
        #b.self.LowerNeckDef.body2	= Neck
        # lower neck
        anchor = k_scale * box2d.b2Vec2(-.015,.575)
        self.LowerNeckDef.localAnchor1	= anchor - self.ChestDef.position
        self.LowerNeckDef.localAnchor2	= anchor - self.NeckDef.position
        self.LowerNeckDef.referenceAngle	= 0.0
        self.LowerNeckDef.lowerAngle		= -0.174532925
        self.LowerNeckDef.upperAngle		= 0.174532925

        #b.self.UpperNeckDef.body1	= Chest
        #b.self.UpperNeckDef.body2	= Head
        # upper neck
        anchor = k_scale * box2d.b2Vec2(-.005,.630)
        self.UpperNeckDef.localAnchor1	= anchor - self.ChestDef.position
        self.UpperNeckDef.localAnchor2	= anchor - self.HeadDef.position
        self.UpperNeckDef.referenceAngle	= 0.0
        self.UpperNeckDef.lowerAngle		= -0.610865238
        self.UpperNeckDef.upperAngle		= 0.785398163

        #b.self.LShoulderDef.body1	= Chest
        #b.self.LShoulderDef.body2	= self.LUpperArm
        #b.self.RShoulderDef.body1	= Chest
        #b.self.RShoulderDef.body2	= self.RUpperArm
        # shoulders
        anchor = k_scale * box2d.b2Vec2(-.015,.545)
        self.LShoulderDef.localAnchor1	= self.RShoulderDef.localAnchor1		= anchor - self.ChestDef.position
        self.LShoulderDef.localAnchor2	= self.RShoulderDef.localAnchor2		= anchor - self.LUpperArmDef.position
        self.LShoulderDef.referenceAngle	= self.RShoulderDef.referenceAngle	= 0.0
        self.LShoulderDef.lowerAngle		= self.RShoulderDef.lowerAngle		= -1.04719755
        self.LShoulderDef.upperAngle		= self.RShoulderDef.upperAngle		= 3.14159265

        #b.self.LElbowDef.body1		= self.LForearm
        #b.self.LElbowDef.body2		= self.LUpperArm
        #b.self.RElbowDef.body1		= self.RForearm
        #b.self.RElbowDef.body2		= self.RUpperArm
        # elbows
        anchor = k_scale * box2d.b2Vec2(-.005,.290)
        self.LElbowDef.localAnchor1		= self.RElbowDef.localAnchor1	= anchor - self.LForearmDef.position
        self.LElbowDef.localAnchor2		= self.RElbowDef.localAnchor2	= anchor - self.LUpperArmDef.position
        self.LElbowDef.referenceAngle	= self.RElbowDef.referenceAngle	= 0.0
        self.LElbowDef.lowerAngle		= self.RElbowDef.lowerAngle		= -2.7925268
        self.LElbowDef.upperAngle		= self.RElbowDef.upperAngle		= 0

        #b.self.LWristDef.body1		= self.LHand
        #b.self.LWristDef.body2		= self.LForearm
        #b.self.RWristDef.body1		= self.RHand
        #b.self.RWristDef.body2		= self.RForearm
        # wrists
        anchor = k_scale * box2d.b2Vec2(-.010,.045)
        self.LWristDef.localAnchor1		= self.RWristDef.localAnchor1	= anchor - self.LHandDef.position
        self.LWristDef.localAnchor2		= self.RWristDef.localAnchor2	= anchor - self.LForearmDef.position
        self.LWristDef.referenceAngle	= self.RWristDef.referenceAngle	= 0.0
        self.LWristDef.lowerAngle		= self.RWristDef.lowerAngle		= -0.174532925
        self.LWristDef.upperAngle		= self.RWristDef.upperAngle		= 0.174532925

    def DefaultPositions(self):
        global k_scale
        for foot in (self.LFootDef, self.RFootDef):
            foot.position		= k_scale * box2d.b2Vec2(-.122,-.901)
        for calf in (self.LCalfDef, self.RCalfDef):
            calf.position		= k_scale * box2d.b2Vec2(-.177,-.771)
        for thigh in (self.LThighDef, self.RThighDef):
            thigh.position		= k_scale * box2d.b2Vec2(-.217,-.391)
        for upperarm in (self.LUpperArmDef, self.RUpperArmDef):
            upperarm.position	= k_scale * box2d.b2Vec2(-.127,.228)
        for forearm in (self.LForearmDef, self.RForearmDef):
            forearm.position    = k_scale * box2d.b2Vec2(-.117,-.011)
        for hand in (self.LHandDef, self.RHandDef):
            hand.position		= k_scale * box2d.b2Vec2(-.112,-.136)

        self.PelvisDef.position	= k_scale * box2d.b2Vec2(-.177,-.101)
        self.StomachDef.position= k_scale * box2d.b2Vec2(-.142,.088)
        self.ChestDef.position	= k_scale * box2d.b2Vec2(-.132,.282)
        self.NeckDef.position	= k_scale * box2d.b2Vec2(-.102,.518)
        self.HeadDef.position	= k_scale * box2d.b2Vec2(.022,.738)


if __name__=="__main__":
     main(Test_Biped)
