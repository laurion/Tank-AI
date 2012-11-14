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

class CollisionProcessing (Framework):
    name="CollisionProcessing"
    def __init__(self):
        super(CollisionProcessing, self).__init__()

        # Ground body
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        sd.friction = 0.3

        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)

        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        xLo = -5.0
        xHi = 5.0
        yLo = 2.0
        yHi = 35.0
        # Small triangle
        triangleShapeDef=box2d.b2PolygonDef()
        triangleShapeDef.vertexCount = 3
        triangleShapeDef.setVertex(0,-1.0, 0.0)
        triangleShapeDef.setVertex(1,1.0, 0.0)
        triangleShapeDef.setVertex(2,0.0, 2.0)
        triangleShapeDef.density = 1.0

        triangleBodyDef=box2d.b2BodyDef() 
        triangleBodyDef.position = (box2d.b2Random(xLo, xHi), box2d.b2Random(yLo, yHi))

        body1 = self.world.CreateBody(triangleBodyDef) 
        body1.CreateShape(triangleShapeDef)
        body1.SetMassFromShapes()

        # Large triangle (recycle definitions)
        triangleShapeDef.setVertex(0, 2.0*triangleShapeDef.getVertex(0))
        triangleShapeDef.setVertex(1, 2.0*triangleShapeDef.getVertex(1))
        triangleShapeDef.setVertex(2, 2.0*triangleShapeDef.getVertex(2))
        triangleBodyDef.position = (box2d.b2Random(xLo, xHi), box2d.b2Random(yLo, yHi))

        body2 = self.world.CreateBody(triangleBodyDef)
        body2.CreateShape(triangleShapeDef)
        body2.SetMassFromShapes()

        # Small box
        boxShapeDef=box2d.b2PolygonDef() 
        boxShapeDef.SetAsBox(1.0, 0.5)
        boxShapeDef.density = 1.0

        boxBodyDef=box2d.b2BodyDef() 
        boxBodyDef.position = (box2d.b2Random(xLo, xHi), box2d.b2Random(yLo, yHi))

        body3 = self.world.CreateBody(boxBodyDef) 
        body3.CreateShape(boxShapeDef)
        body3.SetMassFromShapes()

        # Large box (recycle definitions)
        boxShapeDef.SetAsBox(2.0, 1.0)
        boxBodyDef.position = (box2d.b2Random(xLo, xHi), box2d.b2Random(yLo, yHi))

        body4 = self.world.CreateBody(boxBodyDef)
        body4.CreateShape(boxShapeDef)
        body4.SetMassFromShapes()

        # Small circle
        circleShapeDef=box2d.b2CircleDef() 
        circleShapeDef.radius = 1.0
        circleShapeDef.density = 1.0

        circleBodyDef=box2d.b2BodyDef() 
        circleBodyDef.position = (box2d.b2Random(xLo, xHi), box2d.b2Random(yLo, yHi))

        body5 = self.world.CreateBody(circleBodyDef) 
        body5.CreateShape(circleShapeDef)
        body5.SetMassFromShapes()

        # Large circle
        circleShapeDef.radius *= 2.0
        circleBodyDef.position = (box2d.b2Random(xLo, xHi), box2d.b2Random(yLo, yHi))

        body6 = self.world.CreateBody(circleBodyDef) 
        body6.CreateShape(circleShapeDef)
        body6.SetMassFromShapes()
     
    def Step(self, settings):
        # We are going to destroy some bodies according to contact
        # points. We must buffer the bodies that should be destroyed
        # because they may belong to multiple contact points.
        nuke = []
        
        # Traverse the contact results. Destroy bodies that
        # are touching heavier bodies.
        body_pairs = [(p.shape1.GetBody(), p.shape2.GetBody()) for p in self.points]
        
        for body1, body2 in body_pairs:
            mass1, mass2 = body1.GetMass(), body2.GetMass()
        
            if mass1 > 0.0 and mass2 > 0.0:
                if mass2 > mass1:
                    nuke_body = body1
                else:
                    nuke_body = body2
        
                if nuke_body not in nuke:
                    nuke.append(nuke_body)
        
        # Destroy the bodies, skipping duplicates.
        for b in nuke:
            print "Nuking:", b
            self.world.DestroyBody(b)
        
        nuke = None
     
        super(CollisionProcessing, self).Step(settings)

if __name__=="__main__":
     main(CollisionProcessing)
