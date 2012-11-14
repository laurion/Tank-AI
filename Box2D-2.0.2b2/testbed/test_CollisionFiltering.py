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

class CollisionFiltering (Framework):
    name="CollisionFiltering"
    # This is a test of collision filtering.
    # There is a triangle, a box, and a circle.
    # There are 6 shapes. 3 large and 3 small.
    # The 3 small ones always collide.
    # The 3 large ones never collide.
    # The boxes don't collide with triangles (except if both are small).
    k_smallGroup = 1
    k_largeGroup = -1

    k_defaultCategory = 0x0001
    k_triangleCategory = 0x0002
    k_boxCategory = 0x0004
    k_circleCategory = 0x0008

    k_triangleMask = 0xFFFF
    k_boxMask = 0xFFFF ^ k_triangleCategory
    k_circleMask = 0xFFFF
    def __init__(self):
        super(CollisionFiltering, self).__init__()
        # Ground body
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        sd.friction = 0.3

        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)

        ground = self.world.CreateBody(bd)
        ground.CreateShape(sd)

        # Small triangle
        triangleShapeDef=box2d.b2PolygonDef()
        triangleShapeDef.vertexCount = 3
        triangleShapeDef.setVertex(0,-1.0, 0.0)
        triangleShapeDef.setVertex(1,1.0, 0.0)
        triangleShapeDef.setVertex(2,0.0, 2.0)
        triangleShapeDef.density = 1.0

        triangleShapeDef.filter.groupIndex = self.k_smallGroup
        triangleShapeDef.filter.categoryBits = self.k_triangleCategory
        triangleShapeDef.filter.maskBits = self.k_triangleMask

        triangleBodyDef=box2d.b2BodyDef()
        triangleBodyDef.position = (-5.0, 2.0)

        body1 = self.world.CreateBody(triangleBodyDef)
        body1.CreateShape(triangleShapeDef)
        body1.SetMassFromShapes()

        # Large triangle (recycle definitions)
        triangleShapeDef.setVertex(0, 2.0*triangleShapeDef.getVertex(0))
        triangleShapeDef.setVertex(1, 2.0*triangleShapeDef.getVertex(1))
        triangleShapeDef.setVertex(2, 2.0*triangleShapeDef.getVertex(2))
        triangleShapeDef.filter.groupIndex = self.k_largeGroup
        triangleBodyDef.position = (-5.0, 6.0)
        triangleBodyDef.fixedRotation = True

        body2 = self.world.CreateBody(triangleBodyDef)
        body2.CreateShape(triangleShapeDef)
        body2.SetMassFromShapes()

        # Small box
        boxShapeDef=box2d.b2PolygonDef()
        boxShapeDef.SetAsBox(1.0, 0.5)
        boxShapeDef.density = 1.0

        boxShapeDef.filter.groupIndex = self.k_smallGroup
        boxShapeDef.filter.categoryBits = self.k_boxCategory
        boxShapeDef.filter.maskBits = self.k_boxMask

        boxBodyDef=box2d.b2BodyDef()
        boxBodyDef.position = (0.0, 2.0)

        body3 = self.world.CreateBody(boxBodyDef)
        body3.CreateShape(boxShapeDef)
        body3.SetMassFromShapes()

        # Large box (recycle definitions)
        boxShapeDef.SetAsBox(2.0, 1.0)
        boxShapeDef.filter.groupIndex = self.k_largeGroup
        boxBodyDef.position = (0.0, 6.0)

        body4 = self.world.CreateBody(boxBodyDef)
        body4.CreateShape(boxShapeDef)
        body4.SetMassFromShapes()

        # Small circle
        circleShapeDef=box2d.b2CircleDef()
        circleShapeDef.radius = 1.0
        circleShapeDef.density = 1.0

        circleShapeDef.filter.groupIndex = self.k_smallGroup
        circleShapeDef.filter.categoryBits = self.k_circleCategory
        circleShapeDef.filter.maskBits = self.k_circleMask

        circleBodyDef=box2d.b2BodyDef()
        circleBodyDef.position = (5.0, 2.0)

        body5 = self.world.CreateBody(circleBodyDef)
        body5.CreateShape(circleShapeDef)
        body5.SetMassFromShapes()

        # Large circle
        circleShapeDef.radius *= 2.0
        circleShapeDef.filter.groupIndex = self.k_largeGroup
        circleBodyDef.position = (5.0, 6.0)

        body6 = self.world.CreateBody(circleBodyDef)
        body6.CreateShape(circleShapeDef)
        body6.SetMassFromShapes()

if __name__=="__main__":
     main(CollisionFiltering)
