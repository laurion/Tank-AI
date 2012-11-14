#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/python
#
# Testbed example showing deformable and breakable bodies using the soft
# b2DistanceJoint and a small,liteweight triangle mesher.
# 2008-05-09 / nimodo 
#
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
#

from test_main import *

import TriangleMesh as tm

def H(x):
    return x/2.0
MIN_SQUAREDLENGTH    =0.25  #0.5*0.5

# want the value on restart,
# break joint if reaction exceeds 
g_maxAllowableForce = 200.0

data = {
    'ring_nodes' : (
        (6.00, 3.00),
        (5.12, 5.12),
        (3.00, 6.00),
        (0.88, 5.12),
        (0.00, 3.00),
        (0.88, 0.88),
        (3.00, 0.00),
        (5.12, 0.88),
        (4.50, 3.00),
        (4.06, 4.06),
        (3.00, 4.50),
        (1.94, 4.06),
        (1.50, 3.00),
        (1.94, 1.94),
        (3.00, 1.50),
        (4.06, 1.94)),

    'ring_segments' : (            
        (9, 10),
        (10, 11),
        (11, 12),
        (12, 13),
        (13, 14),
        (14, 15),
        (15, 16),
        (16, 9)),

    'ring_holes' : (
        (3.00, 3.00), ),

    # 'B'
    'B_nodes' : (
        (0.00, 0.00),
        (4.00, 0.00),
        (5.00, 2.00),
        (5.00, 4.00),
        (4.00, 5.00),
        (5.00, 6.00),
        (5.00, 8.00),
        (4.00, 9.00),
        (0.00, 9.00),
        (0.00, 5.00),
        (1.50, 1.50),
        (3.50, 1.50),
        (3.50, 4.00),
        (1.50, 4.00),
        (1.50, 6.00),
        (3.50, 6.00),
        (3.50, 8.50),
        (1.50, 8.50)),

    'B_segments' : (
        (1, 2),
        (2, 3),
        (3, 4),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 8),
        (8, 9),
        (9, 10),
        (10, 1),
        (11, 12),
        (12, 13),
        (13, 14),
        (14, 11),
        (15, 16),
        (16, 17),
        (17, 18),
        (18, 15)),

    'B_holes' : (
        (5.00, 5.00),
        (2.50, 2.50),
        (2.50, 7.00)),

    # 'D'
    'D_nodes' : (
        (0.00, 0.00),
        (4.00, 0.00),
        (5.00, 2.50),
        (5.00, 7.00),
        (4.00, 9.00),
        (0.00, 9.00),
        (0.00, 5.00),
        (1.50, 2.50),
        (3.50, 2.50),
        (3.50, 7.00),
        (1.50, 7.00)),

    'D_segments' : (
        (1, 2),
        (2, 3),
        (3, 4),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 1),
        (8, 9),
        (9, 10),
        (10, 11),
        (11, 8)),

    'D_holes' : (
        (2.50, 5.00), ),

    # 'x'
    'x_nodes' : (
        (0.00, 0.00),
        (1.00, 0.00),
        (5.00, 0.00),
        (6.00, 0.00),
        (6.00, 1.00),
        (6.00, 5.00),
        (6.00, 6.00),
        (1.00, 6.00),
        (5.00, 6.00),
        (0.00, 6.00),
        (0.00, 5.00),
        (0.00, 1.00),
        (3.00, 2.00),
        (4.00, 3.00),
        (3.00, 4.00),
        (2.00, 3.00)),

    'x_segments' : (
        (2, 13),
        (3, 13),
        (5, 14),
        (6, 14),
        (8, 15),
        (9, 15),
        (11, 16),
        (12, 16)),

    'x_holes' : (
        (3.00, 1.00),
        (5.00, 3.00),
        (3.00, 5.00),
        (1.00, 3.00)),

    # '2'
    'two_nodes' : (
        (0.00, 0.00),
        (6.00, 0.00),
        (6.00, 1.00),
        (2.00, 1.00),
        (2.00, 2.00),
        (6.00, 6.00),
        (6.00, 8.00),
        (5.00, 9.00),
        (2.00, 9.00),
        (1.00, 7.50),
        (0.00, 2.50),
        (5.00, 6.50),
        (5.00, 8.00),
        (2.50, 8.00),
        (2.00, 7.50)),

    'two_segments' : (
        (1, 2),
        (2, 3),
        (3, 4),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 8),
        (8, 9),
        (9, 10),
        (10, 15),
        (11, 12),
        (12, 13),
        (13, 14),
        (14, 15)),

    'two_holes' : (
        (3.00, 5.00),
        (4.00, 3.00)),

    # '-' beam
    'beanodes' : (
        (0.00, 0.00),
        (32.00, 0.00),
        (32.00, 3.00),
        (0.00, 3.00)),

    'beasegments' : None,
    'beaholes' : None,

    # 'b' a box
    'b_nodes' : (
        (0.00, 0.00),
        (10.00, 0.00),
        (10.00, 10.00),
        (0.00, 10.00),
        (2.00, 2.00),
        (8.00, 2.00),
        (8.00, 8.00),
        (2.00, 8.00)),

    'b_segments' : (
        (5, 6),
        (6, 7),
        (7, 8),
        (8, 5)),

    'b_holes' : (
        (5.0, 5.0), )

    }

# convert the data to the specific types

for key in data.keys():
    if data[key] == None: continue
    if key[-6:] == "_nodes" or key[-6:] == "_holes":
       data[key] = [tm.tmVertex( (x, y) )   for x, y  in data[key]]
    elif key[-9:] == "_segments":
       data[key] = [tm.tmSegmentId((v0,v1)) for v0,v1 in data[key]]

class BreakableBody(Framework):
    name="BreakableBody"
    maxAllowableForce = 0.0
    drawMode  = False
    drawCount = 0
    drawVertices = []
    staticBodies=False
    _pickle_vars=['maxAllowableForce', 'drawMode', 'drawCount', 'drawVertices',
                  'staticBodies']
    def __init__(self):
        super(BreakableBody, self).__init__()

        if not self.settings.onlyInit:
            self.viewZoom = 6.0

        # geometries
        gx, gy  = 100.0, 1.0
        dx, br  =  34.0, 0.3
        sx, sy  = -dx-H(dx), 30

        # ground
        sd=box2d.b2PolygonDef()
        bd=box2d.b2BodyDef()

        bd.position = (0.0, 0.0)
        ground = self.world.CreateBody(bd)

        # bottom
        sd.SetAsBox( H(gx), H(gy) )
        ground.CreateShape(sd)
        sd.SetAsBox( H(dx), H(gy), (-dx,sy-1.0), 0.0 )
        ground.CreateShape(sd)

        # dyn bodies
        pd=box2d.b2PolygonDef()
        dj=box2d.b2DistanceJointDef()
        
        dj.dampingRatio     = 1.0
        dj.collideConnected = True
       
        nodes, segments, holes = self.ExampleData('B')
        dj.frequencyHz      = 20
        pd.density          = 1.0/70.0
        pd.friction         = 0.4
        pd.restitution      = 0.01
        self.CreateSoftBody( (sx,sy), 0, 0, pd, dj, nodes, segments, holes)
        
        nodes, segments, holes = self.ExampleData('ring')
        dj.frequencyHz      = 20
        pd.density          = 1.0/36.0
        pd.friction         = 0.1
        pd.restitution      = 0.5
        self.CreateSoftBody( (sx+6,sy), 0, 0, pd, dj,nodes, segments, holes)
        
        nodes, segments, holes = self.ExampleData('x')
        dj.frequencyHz      = 0.0
        pd.density          = 0.2
        pd.friction         = 1.0
        pd.restitution      = 0.1
        self.CreateSoftBody( (sx+13,sy),  0,  0,  pd, dj,nodes, segments, holes)
        
        nodes, segments, holes = self.ExampleData('two')
        dj.frequencyHz      = 20.0
        pd.density          = 0.01
        pd.friction         = 0.3
        pd.restitution      = 0.3
        self.CreateSoftBody( (sx+20,sy),  0, 0,   pd, dj, nodes, segments, holes)
        
        nodes, segments, holes = self.ExampleData('D')
        self.CreateSoftBody( (sx+28,sy),  0, 0,   pd, dj, nodes, segments, holes)
        
        nodes, segments, holes = self.ExampleData('b')
        dj.frequencyHz      = 20
        dj.dampingRatio     = 2.0
        pd.restitution      = 0.01
        pd.density          = 0.01
        pd.friction         = 0.9
        self.CreateSoftBody( (-5,5*gy),  0, 0,   pd, dj ,nodes, segments, holes)

        cd=box2d.b2CircleDef()
        bd=box2d.b2BodyDef()
        cd.radius = br
        cd.density= 0.001
        bd.position = (0.0,10.0*gy)
        for i in range(60):
            b = self.world.CreateBody(bd)
            b.CreateShape(cd)
            b.SetMassFromShapes()
    
    # Create compound (soft) body using a triangle mesh
    # If meshDensity is 0, a minimal grid is generated.
    # Actually pd and dj define the behaviour for all triangles
    def CreateSoftBody(self, pos, meshDensity, options, pd, dj,nodes=[], segments=[], holes=[]):
        n_nodes   =len(nodes)
        n_segments=len(segments)
        n_holes   =len(holes) 

        # TriangleMesh defs
        md = tm.TriangleMesh()

        # box2d defs
        bd=box2d.b2BodyDef()

        # in case of meshDensity>3 ...
        md.SetMaxVertexCount(meshDensity)
        if options: md.SetOptions(options)

        # triangulator main
        i = md.Mesh(nodes, segments, holes)
        md.PrintData()
        print "TriangleMesh.py:%s" % md.GetErrorMessage(i)

        # check if enough proxies set in box2d,  10 proxies as reserve
        if self.world.GetProxyCount()+md.GetInsideTriangleCount() > (box2d.b2_maxProxies-10):
            print
            print "Current proxy count:", self.world.GetProxyCount()
            print "World proxy count if added:", self.world.GetProxyCount()+md.GetInsideTriangleCount() 
            print "Max proxies (Box2D setting):", (box2d.b2_maxProxies-10)
            print "Error: Not enough proxies to insert the mesh!"
            md.Reset()
            return

        # bodies (triangles)
        triangles = md.GetTriangles()
        if triangles==None: return
        pd.vertexCount = 3

        for i in range(md.GetTriangleCount()):
            if triangles[i].inside:
                pd.setVertices([triangles[i].v[0],
                                triangles[i].v[1],
                                triangles[i].v[2]])

                try:
                    pd.checkValues()
                except ValueError:
                    print "** Created an invalid shape"
                    exit(0)

                bd.position=pos
                b = self.world.CreateBody(bd)
                b.CreateShape(pd)
                b.SetMassFromShapes()
                # we need the body pointer in the triangles for the joints later
                triangles[i].userData = b

        # joints
        if pd.density>0.0:
            # for each triangle-pair in edges, connect with a distance joint
            edges = md.GetEdges()
            for i in range(md.GetEdgeCount()):
                t0 = edges[i].t[0]
                t1 = edges[i].t[1]
                if t0.inside==False or t1.inside==False: continue
                
                # Get bodies
                b1 = t0.userData
                b2 = t1.userData
                if b1==None or b2==None: continue
                
                dj.Initialize( b1,b2, b1.GetWorldCenter(), b2.GetWorldCenter())
                self.world.CreateJoint(dj)

        # clean TriangleMesh
        md.Reset()
    
    # maybe here to check for maximal reaction forces to break a body
    def Step(self, settings):
        global g_maxAllowableForce
        F=0.0
        jStressed = 0
        for j in self.world.jointList:
            tmp = j.GetReactionForce(settings.hz).Length() # for newer builds
            #tmp = j.GetReactionForce().Length() # works for older builds
            if tmp>F:
                F = tmp
                jStressed = j
        if jStressed and (F>g_maxAllowableForce):
            self.world.DestroyJoint(jStressed)

        self.DrawStringCR("max.reactionforce=%f.0 allowable=%f.0  change:-+" % (F,g_maxAllowableForce));

        onoff = { False: 'off', True: 'on' }
        self.DrawStringCR("(d)rawmode-%s  (m)esh  (s)taticbody-%s" % (onoff[self.drawMode], onoff[self.staticBodies]))

        p1, p2 = box2d.b2Vec2(), box2d.b2Vec2()
        for i in xrange(self.drawCount-1):
            p1 = (self.drawVertices[i].x      ,self.drawVertices[i].y)
            if i<self.drawCount-1:
                p2 = (self.drawVertices[i+1].x,self.drawVertices[i+1].y)
            else:
                p2 = (self.drawVertices[0].x  ,self.drawVertices[0].y)
            self.debugDraw.DrawSegment(p1,p2,box2d.b2Color(0.6,0.2,0.2))
            self.debugDraw.DrawPoint(p1,3.0, box2d.b2Color(0.6,0.5,0.2))

        super(BreakableBody, self).Step(settings)
    
    def Keyboard(self, key):
        global g_maxAllowableForce
        if key==K_MINUS:
            if g_maxAllowableForce > 0.0:
                g_maxAllowableForce -= 5.0
        elif key==K_PLUS or key==K_EQUALS:
            g_maxAllowableForce += 5.0
        elif key==K_d:
            self.drawMode = not self.drawMode
        elif key==K_s:
            self.staticBodies = not self.staticBodies
        elif key==K_m:
            if self.drawCount>0:
                pd = box2d.b2PolygonDef()
                dj = box2d.b2DistanceJointDef()
                dj.collideConnected = True
                dj.frequencyHz      = 20.0
                dj.dampingRatio     = 10.0
                pd.friction         = 0.9
                pd.restitution      = 0.1
                if self.staticBodies:
                    pd.density      = 0.0
                else:
                    pd.density      = 1.0/32.0

                self.CreateSoftBody( box2d.b2Vec2(0.0,0.0),  0, tm.tmO_SEGMENTBOUNDARY|tm.tmO_GRADING|tm.tmO_CHECKINTERSECT,
                    pd, dj, self.drawVertices) 

                self.drawVertices = []
                self.drawCount = 0
                self.drawMode = False

    def AddVertex(self, p):
        if self.drawCount>0:
            dx = self.drawVertices[self.drawCount-1].x - p.x 
            dy = self.drawVertices[self.drawCount-1].y - p.y
            if dx*dx+dy*dy > MIN_SQUAREDLENGTH:
                self.drawVertices.append( box2d.b2Vec2(p.x, p.y) )
                self.drawCount+=1
        else:
            self.drawVertices.append( box2d.b2Vec2(p.x, p.y) )
            self.drawCount+=1

    def MouseDown(self, p):
        if self.drawMode:
            self.AddVertex(p)
        else:
            super(BreakableBody, self).MouseDown(p)

    def MouseUp(self, p):
        if not self.drawMode:
            super(BreakableBody, self).MouseUp(p)
        
    # examples
    def ExampleData(self, which):
        print "\nLoading data:", which

        nodes      = data["%s_nodes" % which]
        segments   = data["%s_segments" % which]
        holes      = data["%s_holes" % which]
        
        return (nodes, segments, holes)

if __name__=="__main__":
    main(BreakableBody)
