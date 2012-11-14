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
class EdgesBuoyancy (Framework):
    name="EdgesBuoyancy"
    def __init__(self):
        super(EdgesBuoyancy, self).__init__()

        bcd = box2d.b2BuoyancyControllerDef() 
        bcd.offset = 15
        bcd.normal = (0,1)
        bcd.density = 2
        bcd.linearDrag = 2
        bcd.angularDrag = 1
        print bcd
        self.bc = self.world.CreateController(bcd)

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
        sd2.density = 0.0 # massless
        
        for i in range(10):
            x = box2d.b2Random(-0.1, 0.1)
            bd=box2d.b2BodyDef()
            bd.position = (x + 5.0, 1.05 + 2.5 * i)
            bd.angle = box2d.b2Random(-box2d.b2_pi, box2d.b2_pi)
            body = self.world.CreateBody(bd)
            body.CreateShape(sd1)
            body.CreateShape(sd2)
            body.SetMassFromShapes()
            self.bc.AddBody(body)
    
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
            self.bc.AddBody(body)
    
        xf1=box2d.b2XForm()
        xf1.R.Set(0.3524 * box2d.b2_pi)
        xf1.position = box2d.b2Mul(xf1.R, (1.0, 0.0))
        
        sd1=box2d.b2PolygonDef()
        sd1.vertexCount = 3
        sd1.setVertex(0, box2d.b2Mul(xf1, (-1.0, 0.0)))
        sd1.setVertex(1, box2d.b2Mul(xf1, (1.0, 0.0)))
        sd1.setVertex(2, box2d.b2Mul(xf1, (0.0, 0.5)))
        sd1.density = 2.0
        
        xf2=box2d.b2XForm()
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
            self.bc.AddBody(body)
    
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
        self.bc.AddBody(body)
    
        loop1=[
            (0.063134534,8.3695248),
            (0.94701801,9.3165428),
            (0.0,9.0640047),
            (-0.12626907,10.326695),
            (1.4520943,11.77879),
            (2.2728432,10.137292),
            (2.3991123,11.147444),
            (3.5986685,10.958041),
            (3.9143411,7.3593722),
            (4.1668793,9.4428119),
            (5.4295699,9.3165428),
            (6.2503189,8.3063903),
            (6.6922606,10.137292),
            (4.9876282,9.8216191),
            (4.7350901,10.958041),
            (7.2604714,11.652521),
            (10.732871,11.147444),
            (10.480333,10.642368),
            (10.732871,9.8216191),
            (11.55362,9.4428119),
            (12.374369,9.3796773),
            (13.005714,9.8216191),
            (13.195118,10.38983),
            (13.005714,10.768637),
            (12.626907,10.894906),
            (12.753176,11.526252),
            (13.573925,11.715655),
            (14.836616,11.399982),
            (16.351844,10.768637),
            (17.867073,11.399982),
            (17.803939,10.263561),
            (17.361997,8.3063903),
            (17.803939,8.1801212),
            (18.056477,9.5059464),
            (18.182746,11.336848),
            (18.561553,11.210579),
            (18.561553,9.6322155),
            (18.561553,7.7381795),
            (18.687822,5.5284708),
            (19.382302,5.6547398),
            (19.066629,8.1801212),
            (19.003495,10.263561),
            (19.066629,11.463117),
            (19.887378,11.841924),
            (20.708127,11.273713),
            (21.0238,10.011023),
            (20.708127,7.2962377),
            (21.086934,6.2860852),
            (21.150069,3.7607038),
            (20.392455,2.5611476),
            (18.624688,2.5611476),
            (20.771262,2.1192059),
            (20.771262,0.22516988),
            (18.624688,-0.2799064),
            (13.826463,0.16203534),
            (14.015867,1.7403987),
            (13.195118,2.1823404),
            (12.626907,1.5509951),
            (12.879445,0.85651522),
            (12.626907,0.35143895),
            (10.543467,1.298457),
            (11.490485,3.9501074),
            (13.889598,3.6344347),
            (13.889598,2.9399549),
            (14.584077,3.8869729),
            (11.932427,5.2127981),
            (9.7227183,4.0132419),
            (10.796005,3.5081657),
            (9.7858528,3.2556275),
            (10.796005,2.4980131),
            (7.9549513,1.7403987),
            (9.6595837,1.424726),
            (9.217642,0.66711162),
            (8.270624,-0.090502792),
            (7.0079333,0.85651522),
            (6.1240498,-0.15363733),
            (6.1240498,3.192493),
            (5.6821081,2.4348786),
            (4.9876282,2.1192059),
            (4.1037447,1.8666678),
            (3.0304576,1.8666678),
            (2.0834396,2.245475),
            (1.6414979,2.6242822),
            (1.3258252,3.5081657),
            (1.2626907,0.47770802),
            (0.63134534,0.035766276),
            (0.063134534,0.9827842),
            ]
        loop2 = [
            (8.270624,6.1598161),
            (8.270624,5.3390672),
            (8.7757003,5.086529),
            (9.4701801,5.5284708),
            (9.217642,6.033547),
            (8.7757003,6.412354),
            ]
        
        b2Loop1 = [(loop[0] + 10.0, loop[1]+1.0) for loop in loop1]
        b2Loop2 = [(loop[0] - 10.0, loop[1]) for loop in loop2]

        b2Loop1.reverse()
        
        bd=box2d.b2BodyDef()
        bd.position = ( 0.0, 0.0 )
        body = self.world.CreateBody(bd)
        
        weight=box2d.b2CircleDef()
        weight.filter.maskBits = 0
        weight.density = 4.0
        weight.radius = 0.5
        weight.localPosition = (8.9, 5.75)
        body.CreateShape(weight)
        
        edgeDef=box2d.b2EdgeChainDef()
        edgeDef.setVertices(b2Loop2)
        body.CreateShape(edgeDef)
        
        body.SetMassFromShapes()
        #self.bc.AddBody(body)
        # edges and buoyancy aren't quite working yet
        # you can uncomment this and see what happens
        
        body = self.world.CreateBody(bd)
        weight.radius = 5.0
        weight.localPosition = (20.5, 7.0)
        body.CreateShape(weight)
        
        edgeDef.setVertices(b2Loop1)
        body.CreateShape(edgeDef)
        
        body.SetMassFromShapes()
        #self.bc.AddBody(body)

if __name__=="__main__":
    main(EdgesBuoyancy)
