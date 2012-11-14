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
class ApplyForce (Framework):
    name="ApplyForce"
    body=None
    _pickle_vars=['body']
    def __init__(self):
        super(ApplyForce, self).__init__()
        self.world.SetGravity((0.0, 0.0))

        k_restitution = 0.4

        bd=box2d.b2BodyDef() 
        bd.position = (0.0, 20.0)
        ground = self.world.CreateBody(bd) 

        sd=box2d.b2PolygonDef() 
        sd.density = 0.0
        sd.restitution = k_restitution

        sd.SetAsBox(0.2, 20.0, (-20.0, 0.0), 0.0)
        ground.CreateShape(sd)

        sd.SetAsBox(0.2, 20.0, (20.0, 0.0), 0.0)
        ground.CreateShape(sd)

        sd.SetAsBox(0.2, 20.0, (0.0, -20.0), 0.5 * box2d.b2_pi)
        ground.CreateShape(sd)

        sd.SetAsBox(0.2, 20.0, (0.0, 20.0), -0.5 * box2d.b2_pi)
        ground.CreateShape(sd)

        xf1 = box2d.b2XForm ()
        xf1.R.Set(0.3524 * box2d.b2_pi)
        xf1.position = box2d.b2Mul(xf1.R, (1.0, 0.0))

        sd1=box2d.b2PolygonDef() 
        sd1.vertexCount = 3
        sd1.setVertex(0, box2d.b2Mul(xf1, (-1.0, 0.0)))
        sd1.setVertex(1, box2d.b2Mul(xf1, (1.0, 0.0)))
        sd1.setVertex(2, box2d.b2Mul(xf1, (0.0, 0.5)))
        sd1.density = 2.0

        xf2 = box2d.b2XForm ()
        xf2.R.Set(-0.3524 * box2d.b2_pi)
        xf2.position = box2d.b2Mul(xf2.R, (-1.0, 0.0))

        sd2=box2d.b2PolygonDef() 
        sd2.vertexCount = 3
        sd2.setVertex(0, box2d.b2Mul(xf2, (-1.0, 0.0)))
        sd2.setVertex(1, box2d.b2Mul(xf2, (1.0, 0.0)))
        sd2.setVertex(2, box2d.b2Mul(xf2, (0.0, 0.5)))
        sd2.density = 2.0

        bd=box2d.b2BodyDef() 
        bd.angularDamping = 2.0
        bd.linearDamping = 0.1

        bd.position = (0.0, 1.05)
        bd.angle = box2d.b2_pi
        self.body = self.world.CreateBody(bd)
        self.body.CreateShape(sd1)
        self.body.CreateShape(sd2)
        self.body.SetMassFromShapes()
     
    def Keyboard(self, key):
        if not self.body:
            return

        if key==K_w:
            f = self.body.GetWorldVector((0.0, -200.0))
            p = self.body.GetWorldPoint((0.0, 2.0))
            self.body.ApplyForce(f, p)
        elif key==K_a:
            self.body.ApplyTorque(20.0)
        elif key==K_d:
            self.body.ApplyTorque(-20.0)

if __name__=="__main__":
     main(ApplyForce)
