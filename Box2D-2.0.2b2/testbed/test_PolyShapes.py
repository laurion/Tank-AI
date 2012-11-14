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

class PolyShapes (Framework):
    name="PolyShapes"
    bodyIndex=0
    bodies=[]
    sds=[]
    circleDef=None
    _pickle_vars=['bodies']

    def __init__(self):
        super(PolyShapes, self).__init__()
        # Ground body
        sds = self.sds

        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(50.0, 10.0)
        sd.friction = 0.3

        for i in range(4):
            sds.append(box2d.b2PolygonDef())

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, -10.0)
        ground = self.world.CreateBody(bd) 
        ground.CreateShape(sd)
        sds[0].vertexCount = 3
        sds[0].setVertex(0,-0.5, 0.0)
        sds[0].setVertex(1,0.5, 0.0)
        sds[0].setVertex(2,0.0, 1.5)
        sds[0].density = 1.0
        sds[0].friction = 0.3

        sds[1].vertexCount = 3
        sds[1].setVertex(0,-0.1, 0.0)
        sds[1].setVertex(1,0.1, 0.0)
        sds[1].setVertex(2,0.0, 1.5)
        sds[1].density = 1.0
        sds[1].friction = 0.3

        sds[2].vertexCount = 8
        w = 1.0
        b = w / (2.0 + math.sqrt(2.0))
        s = math.sqrt(2.0) * b
        sds[2].setVertex(0,0.5 * s, 0.0)
        sds[2].setVertex(1,0.5 * w, b)
        sds[2].setVertex(2,0.5 * w, b + s)
        sds[2].setVertex(3,0.5 * s, w)
        sds[2].setVertex(4,-0.5 * s, w)
        sds[2].setVertex(5,-0.5 * w, b + s)
        sds[2].setVertex(6,-0.5 * w, b)
        sds[2].setVertex(7,-0.5 * s, 0.0)
        sds[2].density = 1.0
        sds[2].friction = 0.3

        sds[3].SetAsBox(0.5, 0.5)
        #sds[3].vertexCount = 4
        #sds[3].setVertex(0,-0.5, 0.0)
        #sds[3].setVertex(1,0.5, 0.0)
        #sds[3].setVertex(2,0.5, 1.0)
        #sds[3].setVertex(3,-0.5, 1.0)
        sds[3].density = 1.0
        sds[3].friction = 0.3

        self.circleDef=box2d.b2CircleDef()
        self.circleDef.radius = 0.5
        self.circleDef.density = 1.0

        self.bodyIndex = 0

    def Create(self, index):
          bd=box2d.b2BodyDef() 
          
          x = box2d.b2Random(-2.0, 2.0)
          bd.position = (x, 10.0)
          bd.angle = box2d.b2Random(-box2d.b2_pi, box2d.b2_pi)
          
          if index == 4:
              bd.angularDamping = 0.02
          
          self.bodies.append(self.world.CreateBody(bd) )
          
          if index < 4:
              self.bodies[-1].CreateShape(self.sds[index])
          else:
              self.bodies[-1].CreateShape(self.circleDef)
          self.bodies[-1].SetMassFromShapes()
          
    def DestroyBody(self):
         for body in self.bodies:
            self.bodies.remove(body)
            self.world.DestroyBody(body)
            return
     
    def Keyboard(self, key):
        if key==K_1 or key==K_2 or key==K_3 or key==K_4 or key==K_5:
            self.Create(key - K_1)

        elif key==K_d:
            self.DestroyBody()
     
    def Step(self, settings):
        super(PolyShapes, self).Step(settings)
        self.DrawStringCR("Press 1-5 to drop stuff")
     
if __name__=="__main__":
     main(PolyShapes)
