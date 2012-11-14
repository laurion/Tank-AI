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
from math import atan2

# C++ version by Smartcode
# See http://www.box2d.org/forum/viewtopic.php?f=3&t=2280

class Belt (Framework):
    name="Belt"
    
    def __init__(self):
        super(Belt, self).__init__()
        bd=box2d.b2BodyDef()
        
        # static body
        bd.position=(0.0, 0.0)
        stat_c = self.world.CreateBody(bd)
        
        # first circle
        bd.position=(-5, 10)
        c1 = self.world.CreateBody(bd)

        sd=box2d.b2CircleDef()
        sd.radius = 4.3
        sd.density = 3.0
        sd.friction = 0.8
        sd.restitution = 0.4
        c1.CreateShape(sd)
        c1.SetMassFromShapes()
        
        # first joint
        rjd=box2d.b2RevoluteJointDef()
        rjd.Initialize(stat_c, c1, c1.GetWorldCenter())
        self.world.CreateJoint(rjd)
        
        # second circle
        bd.position=(10, 15)
        c2 = self.world.CreateBody(bd)
        sd.radius = 2.2
        c2.CreateShape(sd)
        c2.SetMassFromShapes()
        
        # second joint
        rjd.Initialize(stat_c, c2, c2.GetWorldCenter())
        self.world.CreateJoint(rjd)
        
        # create belt from the table of points
        tuple_points = [
            (-5.97,  14.91),  (-3.17,  15.46),  (-0.38,  16.01),  ( 2.42,  16.56),  ( 5.22,  17.11),
            (8.01,  17.67 ),  (10.82,  17.88),  (12.83,  16.01),  (12.46,  13.29),  (10.12,  11.73),
            (7.56,  10.49 ),  ( 4.99,   9.26),  ( 2.42,   8.02),  (-0.15,   6.79),  (-2.72,   5.55),
            (-5.48,   5.02),  (-8.09,   6.07),  (-9.72,   8.36),  (-9.86,  11.17),  (-8.46,  13.61) ]
        
        points = [box2d.b2Vec2(*t) for t in tuple_points]
        
        pd=box2d.b2PolygonDef()
        jd=box2d.b2RevoluteJointDef()
        
        pd.density = 3.0
        pd.friction = 0.8
        
        thickness = 1.0

        first_body= None
        prev_body = None
        next_body = None
        for i in range(20):
            v = points[(i+1)%20] - points[i]
            angle = atan2(v.y, v.x)
            
            bd.position = points[i] + 0.5 * v

            next_body = self.world.CreateBody(bd)

            if i == 0:
                first_body = next_body # remember the body of first fragment
            
            pd.SetAsBox(0.5 * v.Length() + thickness / 2.0, thickness / 2.0, (0,0), angle)

            shape = next_body.CreateShape(pd)
            next_body.SetMassFromShapes()
            
            if prev_body: # if we have previous body, create joint
                local_anchor=points[i]
                jd.Initialize(prev_body, next_body, local_anchor)
                self.world.CreateJoint(jd)
            
            prev_body = next_body
        
        # last anchor between last and first element
        jd.Initialize(prev_body, first_body, points[0])
        self.world.CreateJoint(jd)

if __name__=="__main__":
    main(Belt)
