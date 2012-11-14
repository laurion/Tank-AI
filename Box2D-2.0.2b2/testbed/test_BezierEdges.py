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

def calculate_bezier(p, steps = 30):
    """
    Calculate a bezier curve from 4 control points and return a 
    list of the resulting points.
    
    The function uses the forward differencing algorithm described here: 
    http://www.niksula.cs.hut.fi/~hkankaan/Homepages/bezierfast.html
    """
    
    t = 1.0 / steps
    temp = t*t
    
    f = p[0]
    fd = 3 * (p[1] - p[0]) * t
    fdd_per_2 = 3 * (p[0] - 2 * p[1] + p[2]) * temp
    fddd_per_2 = 3 * (3 * (p[1] - p[2]) + p[3] - p[0]) * temp * t
    
    fddd = fddd_per_2 + fddd_per_2
    fdd = fdd_per_2 + fdd_per_2
    fddd_per_6 = fddd_per_2 * (1.0 / 3)
    
    points = []
    for x in range(steps):
        points.append(f)
        f = f + fd + fdd_per_2 + fddd_per_6
        fd = fd + fdd + fddd_per_2
        fdd = fdd + fddd
        fdd_per_2 = fdd_per_2 + fddd_per_2
    points.append(f)
    return points

class BezierEdges (Framework):
    name="BezierEdges"
    control_points = [ ]
    bezier_obj = []
    selected_point = 0
    _pickle_vars=['control_points', 'bezier_obj', 'selected_point']

    def __init__(self):
        super(BezierEdges, self).__init__()

        self.control_points = [ box2d.b2Vec2(*t) for t in
                                    [(0,0), (0,1), (0, 2), (0, 3)]]

        # ground edge
        verts = [ (50.0,0.0), (-50.0,0.0) ]
        edgeDef = box2d.b2EdgeChainDef()
        edgeDef.setVertices_tuple(verts)
        edgeDef.isALoop = False

        bd=box2d.b2BodyDef()
        bd.position = ( 0.0, 0.0 )
        ground = self.world.CreateBody(bd)
        ground.CreateShape(edgeDef)

        # some objects to use on the curve
        sd=box2d.b2CircleDef()
        sd.radius = 1.0
        sd.density = 1.0
        
        for i in xrange(4):
            bd=box2d.b2BodyDef()
            bd.position = (0.0, 2.0 + 3.0 * i)
            
            body = self.world.CreateBody(bd)
            body.CreateShape(sd)
            body.SetMassFromShapes()
        
    def Keyboard(self, key):
        if key==K_c:
            # create a new curve
            bezier_verts = calculate_bezier(self.control_points)
            
            ed = box2d.b2EdgeChainDef()
            ed.setVertices_b2Vec2(bezier_verts)
            ed.isALoop = False

            bd=box2d.b2BodyDef()
            bd.position = ( 0.0, 0.0 )
            body = self.world.CreateBody(bd)
            body.CreateShape(ed)

            self.bezier_obj.append(body)
        elif key==K_d:
            # delete the most recent curve
            if self.bezier_obj:
                body = self.bezier_obj.pop()
                self.world.DestroyBody( body )
    
        elif key in (K_1, K_2, K_3, K_4):
            self.selected_point = key-K_1

    def ShiftMouseDown(self, p):
        self.control_points[self.selected_point]=p
        self.selected_point += 1
        if self.selected_point > 3: 
            self.selected_point = 0

    def Step(self, settings):
        super(BezierEdges, self).Step(settings)

        for p in self.control_points:
            self.debugDraw.DrawCircle(p, 0.25, box2d.b2Color(1.0,0.0,0))

if __name__=="__main__":
    main(BezierEdges)

