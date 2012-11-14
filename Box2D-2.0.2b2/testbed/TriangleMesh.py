#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# A lite constrained delauney triangle mesh generator,supporting holes and
# non convex boundaries.
# (c) 2008 nimodo@hispeed.ch
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
# Python changelog:
#  Updated to SVN r168 on 7/25
#  Updated to 08.08.2008 version from http://www.box2d.org/forum/viewtopic.php?f=3&t=836
#
# -----------------------------------------------------------------------------
# Original comments (ported from the C++ version):
#
# See
# 1) A Delaunay Refinement Algorithm for Quality 2-Dimensional Mesh Generation
#    Jim Ruppert - Journal of Algorithms, 1995
# 2) Jonathan Shewchuk
#    http://www.cs.cmu.edu/~quake/triangle.html
# 3) Recursive triangle eating
#    Francois Labelle
#    http://www.cs.berkeley.edu/~flab/
# 4) example - at the end of this file
#
#
# 2001/03 as part of a basic FEA package
# 2008/05 some small changes for box2d
# 2008/06 small bugs
#         tmO_BASICMESH option for testing only, works sometimes ;)
#         some comments (see .h file)
#         variable names changed for better understanding, example
#          - tmO_MINIMALGRID renamed to tmO_GRADING, used option with
#            gradingLowerAngle 
#         bug in SegmentVertices()
#         playing with "zero tolerances..."
#         GetVersion() added
# 2008/07 tmO_CHECKINTERSEC (optional) added, see HasIntersections()
#         alternative to Mesh() added, using tmVertex* instead of tmSegmentId*
#         malloc() replaced with Alloc()
#         zlib license added
#

from math import sin, cos, atan2, sqrt, pow, floor, log
from test_main import box2d

# errors and warnings
tmE_OK               =0
tmE_MEM              =1
tmE_HOLES            =2
tmE_NOINSIDETRIANGLES=3
tmE_INTERSECTS       =4
tmE_OPENFILEWT       =5

# constants
tmC_MAXVERTEXCOUNT  =500
tmVERSION           =1.004
tmC_ZEROTOL         =0.00001
tmC_PI              =3.14159265359
tmC_PIx2            =6.28318530718
tmC_PI_3            =1.04719755119
tmC_SQRT2           =1.41421356237
# default big number to calculate a triangle covering all points (vertices[0-2])
tmC_BIGNUMBER       =1.0e10
#default maximal number of vertices (resp. nodes)
tmC_DEFAULTMAXVERTEX=500
# default abort-inserting if tmO_GRADING option set (angle in deg)
tmC_DEFAULTGRADINGLOWERANGLE   =30.0


# TriangleMesh.options
# automatic segment boundary vertices => SegmentVertices()
tmO_SEGMENTBOUNDARY=  2
# hull vertices => ConvexHull()
tmO_CONVEXHULL     =  4
# abort (=>InsertSegments()), if worst angle > minAngle
tmO_MINIMALGRID    =  8  # deprecated
tmO_GRADING        =  8
# turn on intesection check => HasIntersections()
tmO_CHECKINTERSECT = 16
# bits for playing... around,debugging and testing, see code
tmO_BASICMESH      = 64
tmO_NOCALC         =128
tmO_BASICMESHNODEL =256
tmO_WRITEINPUT     =512 

tmErrorMessages = { tmE_OK: "ok",
                    tmE_MEM: "memory allocation failed",
                    tmE_HOLES: "could not drill the holes",
                    tmE_NOINSIDETRIANGLES: "there are no inside triangles,all might be eaten",
                    tmE_INTERSECTS: "intersecting boundary segments found" }

class tmVertex(box2d.b2Vec2):
    # x, y
    def __init__(self, tuple=(0.0, 0.0)):
        super(tmVertex, self).__init__()
        self.x, self.y = tuple

    def __repr__(self):
        return "(%f, %f)" % (self.x, self.y)

class tmSegment(object):
    # tmVertex v[2] 
    def __init__(self):
        self.v = [tmVertex(), tmVertex()]

class tmSegmentId(object):
    # i1, i2
    def __init__(self, tuple=(0, 0)):
        self.i1, self.i2 = tuple

class tmEdge(object):
    def __init__(self):
        self.v = [None, None]
        self.locked = False
        self.t = [None, None]

class Triangle(object):
    minAngle = 0.0
    angle    = 0.0
    inside   = False
    # hold attributes for the triangles, external use only
    userData = None
    area     = 0.0
    def __init__(self):
        self.v = [None, None, None]
        self.e = [tmEdge(), tmEdge(), tmEdge()]

class TriangleMesh(object):
    Vertices  = []
    Edges     = []
    Triangles = []
    Segments  = []
    gradingLowerAngle = tmC_DEFAULTGRADINGLOWERANGLE
    maxVertexCount=0
    maxEdgeCount=0
    maxTriangleCount=0
    maxSegmentCount=0
    vertexCount=0
    inputVertexCount=0
    edgeCount=0
    triangleCount=0
    segmentCount=0
    holeCount=0
    insideTriangleCount=0
    haveEnoughVertices=False

    options=0

    lastTriangle = None
    lastErrorMessage = ""

    # defaults at instancing
    def __init__(self, aMaxVertexCount=tmC_MAXVERTEXCOUNT, aOptions=tmO_MINIMALGRID|tmO_CONVEXHULL):
        self.Reset()
        self.maxVertexCount = aMaxVertexCount
        self.options        = aOptions

    # set-funtions
    def SetMaxVertexCount(self, count):
       if count>3:
           self.maxVertexCount = count
           self.options &= ~tmO_GRADING

    def SetOptions(self, options):   self.options = options
    def AddOption(self, options):    self.options |= options
    def DeleteOption(self, options): self.options &= ~options
    def SetGradingLowerAngle(self, angle):
        self.gradingLowerAngle = angle
        self.options |= tmO_GRADING

    # get-functions
    def GetVertexCount(self):          return self.vertexCount        
    def GetInputVertexCount(self):     return self.inputVertexCount   
    def GetEdgeCount(self):            return self.edgeCount          
    def GetTriangleCount(self):        return self.triangleCount      
    def GetSegmentCount():             return self.segmentCount
    def GetHoleCount():                return self.holeCount
    def GetInsideTriangleCount(self):  return self.insideTriangleCount
    def GetVertices(self):  return self.Vertices  
    def GetEdges(self):     return self.Edges     
    def GetTriangles(self): return self.Triangles  
    def GetSegments():      return self.Segments
    def GetHoles():         return self.Holes
    
    # main mesh function
    def Mesh(self, input, segment, hole):
        # check segment type -- two separate functions for each
        if len(segment) > 0 and isinstance(segment[0], tmVertex):
            return self.MeshSV(input, segment, hole)
        # otherwise, it's hopefully a tmSegmentID

        endVertex, rtn = self.Setup(input, segment, hole)
         
        if rtn!=tmE_OK:
            return rtn

        # for testing only
        if self.options&tmO_NOCALC:
            return 0

        # check intersections
        if self.options&tmO_CHECKINTERSECT:
            if self.HasIntersections(input, 0, endVertex):
                return tmE_INTERSECTS

        # mesh main
        return self.DoMesh(len(input))

    # secondary mesh function (for segments passed as points)
    def MeshSV(self, input, segment, hole):
        n_input  =len(input)
        n_segment=len(segment)
        n_holes  =len(hole)

        sid = []
        v   = []

        # alloc space
        for i in xrange(n_segment):
            sid.append(tmSegmentId())
        for i in xrange(n_input + n_segment):
            v.append(tmVertex())

        # copy points and assign seg id's
        for i in xrange(0, n_input):
            v[i].x = input[i].x
            v[i].y = input[i].y
        for i in xrange(n_input, n_input+n_segment):
            v[i].x = segment[i-n_input].x
            v[i].y = segment[i-n_input].y

        for i in (0,n_segment-1):
            sid[i].i1 = n_input+i+1
            sid[i].i2 = n_input+i+2

        sid[n_segment-1].i1 = n_input+n_segment
        sid[n_segment-1].i2 = n_input+1

        # setup data
        endVertex, rtn = self.Setup(v, sid, hole)
         
        if rtn!=tmE_OK:
            return rtn

        # for testing only
        if self.options&tmO_NOCALC:
            return 0

        # check intersections
        if self.options&tmO_CHECKINTERSECT:
            if self.HasIntersections(input, 0, endVertex):
                return tmE_INTERSECTS

        # mesh main
        return self.DoMesh(n_input+n_segment)

    def DoMesh(self, n_input):
        rtn = tmE_OK
        hasInsideTriangles=False

        self.Triangulate()

        if self.options & tmO_BASICMESH ==0:
            self.inputVertexCount = self.vertexCount
            # non convex graphs
            if self.options & tmO_CONVEXHULL:
                self.ConvexHull()

            self.InsertSegments()
            # mark triangles
            if self.haveEnoughVertices:
               self.MarkInsideTriangles(True)
               
               for i in range(self.triangleCount):
                    if self.Triangles[i].inside:
                         hasInsideTriangles = True
                         break
               if not hasInsideTriangles:
                   return tmE_NOINSIDETRIANGLES
            else:
               self.MarkInsideTriangles(False)
            #
            for i in range(self.segmentCount):
               e  = self.GetEdge(self.Segments[i].v[0], self.Segments[i].v[1])
               if e: e.locked = True
            #
            self.DeleteBadTriangles()
            # debug - for testing purposes only !
        else:
            # for testing only
            # quick & dirty hack for a mesh with lesser angles than with the
            # tmO_GRADING flag and gradingLowerAngle set
            self.MarkInsideTriangles( not (self.options&tmO_BASICMESHNODEL) )

        # restore original number of input vertices
        self.inputVertexCount = n_input

        # count inner triangles
        self.insideTriangleCount = 0
        for i in range(self.triangleCount):
            if self.Triangles[i].inside:
                self.insideTriangleCount += 1

        return rtn

    def Setup(self, input, segment, holes):
        rtn = tmE_OK

        n_input  =len(input)
        n_segment=len(segment)
        n_holes  =len(holes)

        self.inputVertexCount = n_input
        self.vertexCount      = n_input + 3

        # max sizes
        if n_input>self.maxVertexCount:
            self.maxVertexCount = n_input
        self.maxVertexCount  += 3
        self.maxEdgeCount     = 3*self.maxVertexCount - 6
        self.maxTriangleCount = 2*self.maxVertexCount - 5 + 1
        self.maxSegmentCount  = 3*self.maxVertexCount - 6

        # allocate space
        for i in xrange(self.maxVertexCount):
            self.Vertices.append(tmVertex())
        for i in xrange(self.maxEdgeCount):
            self.Edges.append(tmEdge())
        for i in xrange(self.maxTriangleCount):
            self.Triangles.append(Triangle())
        for i in xrange(self.maxSegmentCount):
            self.Segments.append(tmSegment())

        # first 3 points make a big equilateral triangle
        for i in range(3):
            self.Vertices[i].x = tmC_BIGNUMBER * cos(i*(tmC_PIx2/3.0))
            self.Vertices[i].y = tmC_BIGNUMBER * sin(i*(tmC_PIx2/3.0))

        # copy input vertices
        if input and n_input>0:
            for i in range(3,self.vertexCount):
                self.Vertices[i].x = input[i-3].x
                self.Vertices[i].y = input[i-3].y

        # add boundary and close last/first,this adds ALL input vertices but
        # to the first input segment
        endVertex = self.inputVertexCount
        if self.options & tmO_SEGMENTBOUNDARY:
            # find outer boundary end-node, assume first segment input is start
            # of inner boundaries (holes)
            if n_segment>0:
                if segment[0].i1<self.inputVertexCount and segment[0].i2==segment[0].i1+1:
                    endVertex = segment[0].i1-1
            self.SegmentVertices(1, endVertex, True)

        # given segments
        if n_segment>0:
            k = 0
            for i in xrange(self.segmentCount+n_segment):
                self.Segments[i].v[0] = self.Vertices[segment[k].i1+3-1]
                self.Segments[i].v[1] = self.Vertices[segment[k].i2+3-1]
                k += 1
            self.segmentCount += n_segment

        # assign hole pointer
        self.holeCount = n_holes
        self.Holes     = holes

        if self.options & tmO_WRITEINPUT:
            self.WriteInput(segment)

        return endVertex, rtn
    def Intersect(self, v0, v1, w0, w1):
        # check consecutive vertices
        if v0==w1 or v1==w0:
            return False

        # test v for intersection
        d1 = self.GetVertexPosition(v0, v1, w0)
        d2 = self.GetVertexPosition(v0, v1, w1)
        if d1*d2 > 0.0:
            return False    # same sign

        # test w for intersection
        d1 = self.GetVertexPosition(w0, w1, v0)
        d2 = self.GetVertexPosition(w0, w1, v1)
        if d1*d2 > 0.0:
            return False    # same sign

        # intersection
        return True 

    def HasIntersections(self, v, start, end):
        for i in range(start, end):
            if i < end-1:
                i1 = i+1
            else:
                i1 = start

            for k in range(i+1, end):
                if k < end-1:
                    k1 = k+1
                else:
                    k1 = start
                if self.Intersect( v[i],v[i1], v[k],v[k1] ):
                    return True
        return False

    def Triangulate(self):
         self.triangleCount = 0
         self.edgeCount     = 0
         self.lastTriangle  = None
         
         v0 = self.Vertices[0]
         v1 = self.Vertices[1]
         v2 = self.Vertices[2]
         
         t0 = self.AddTriangle()
         t1 = self.AddTriangle()
         
         e0 = self.AddEdge()
         e1 = self.AddEdge()
         e2 = self.AddEdge()
         
         self.SetTriangle( t0, v0, v1, v2, e0, e1, e2)
         self.SetTriangle( t1, v0, v2, v1, e2, e1, e0)
         
         self.SetEdge(e0, v0, v1, t0, t1)
         self.SetEdge(e1, v1, v2, t0, t1)
         self.SetEdge(e2, v2, v0, t0, t1)
         
         for i in range(3, self.vertexCount):
             self.InsertVertex(self.Vertices[i])

    def CircumCenter(self, c, t):
         # center
         c0x = (t.v[0].x + t.v[1].x + t.v[2].x)/3.0
         c0y = (t.v[0].y + t.v[1].y + t.v[2].y)/3.0

         # deltas
         dx  = t.v[1].x - t.v[0].x
         dy  = t.v[1].y - t.v[0].y
         ex  = t.v[2].x - t.v[0].x
         ey  = t.v[2].y - t.v[0].y
         #
         f   = 0.5 / (ex*dy - ey*dx)
         e2  = ex*ex + ey*ey
         d2  = dx*dx + dy*dy
         c1x = t.v[0].x + f * (e2*dy - d2*ey)
         c1y = t.v[0].y + f * (d2*ex - e2*dx)

         # look if already existing
         for i in range(20):
              c.x = c1x
              c.y = c1y
              if self.FindVertex(c):
                   v = self.GetClosestVertex(c1x, c1y)
                   if (v==t.v[0] or v==t.v[1] or v==t.v[2]): return
              c1x = c0x + 0.9*(c1x-c0x)
              c1y = c0y + 0.9*(c1y-c0y)

         # center
         c.x = c0x
         c.y = c0y

    def DeleteTriangle(self, t):
        # delete recursive
         if t.inside==False: return 

         t.inside = False
         for i in range(3):
            e = t.e[i]
            if self.GetSegment( e.v[0], e.v[1])==None:
                if   e.t[0]==t: self.DeleteTriangle(e.t[1])
                elif e.t[1]==t: self.DeleteTriangle(e.t[0])
                else: assert( e.t[0]==t or e.t[1]==t )

    def SegmentVertices(self, startNode, endNode, doclose):
         k = self.segmentCount
         for i in range(startNode-1, endNode-1):
             k+=1
             self.Segments[k].v[0] = self.Vertices[i+3]
             self.Segments[k].v[1] = self.Vertices[i+3+1]
             self.segmentCount+=1

         if doclose:
             self.Segments[k].v[0] = self.Vertices[i+3]
             self.Segments[k].v[1] = self.Vertices[3]
             self.segmentCount+=1

    def AddVertex(self):
         if self.vertexCount >= self.maxVertexCount: return None
         self.vertexCount+=1
         return self.Vertices[self.vertexCount-1]

    def AddEdge(self):
         assert( self.edgeCount<self.maxEdgeCount )
         self.edgeCount+=1
         return self.Edges[self.edgeCount-1]

    def AddSegment(self):
         assert( self.segmentCount<self.maxSegmentCount )
         self.segmentCount+=1
         return self.Segments[self.segmentCount-1]

    def AddTriangle(self):
         assert(self.triangleCount < self.maxTriangleCount)
         self.Triangles[self.triangleCount].userData = None
         self.triangleCount += 1
         return self.Triangles[self.triangleCount-1]

    def SameVertex(self, v0, v1):
        return (    abs(v0.x - v1.x) < tmC_ZEROTOL 
                and abs(v0.y - v1.y) < tmC_ZEROTOL )

    def GetOppositeVertex(self, e, t):
        if e==t.e[0]: return t.v[2]
        if e==t.e[1]: return t.v[0]
        if e==t.e[2]: return t.v[1]
        return None 

    def SetEdge(self, e, v0, v1, t0, t1):
        e.v[0], e.v[1] = v0, v1
        e.t[0], e.t[1] = t0, t1
        e.locked=False

    def GetEdge(self, v0, v1):
         for i in range(self.edgeCount):
              if ((v0==self.Edges[i].v[0] and v1==self.Edges[i].v[1]) or
                 (v0==self.Edges[i].v[1] and v1==self.Edges[i].v[0])):
                    return self.Edges[i]
         return None

    def SetTriangle(self,t,v0,v1,v2,e0,e1,e2) :
        t.v[0], t.v[1], t.v[2] = v0, v1, v2
        t.e[0], t.e[1], t.e[2] = e0, e1, e2
        t.minAngle, t.angle, t.area = self.SetTriangleData( v0, v1, v2)
        t.inside = True

    def SetTriangleData(self,v0,v1,v2):
        d0x = v1.x - v0.x; d0y = v1.y - v0.y
        d1x = v2.x - v1.x; d1y = v2.y - v1.y
        d2x = v0.x - v2.x; d2y = v0.y - v2.y
        
        t0 = self.ArcTan2(d0y, d0x)
        t1 = self.ArcTan2(d1y, d1x)
        t2 = self.ArcTan2(d2y, d2x)
        
        a0 = self.GetAngle(t2 + tmC_PI, t0)
        a1 = self.GetAngle(t0 + tmC_PI, t1)
        a2 = self.GetAngle(t1 + tmC_PI, t2)

        amin = min(a0, a1, a2)

        minAngle = amin*180.0/tmC_PI
        
        if self.IsOppositeVertex( v2, v0, v1):  a0 = tmC_PI_3 
        if self.IsOppositeVertex( v0, v1, v2):  a1 = tmC_PI_3
        if self.IsOppositeVertex( v1, v2, v0):  a2 = tmC_PI_3

        amin = min(a0, a1, a2)
        
        if self.options & tmO_GRADING:
             angle = amin*180.0/tmC_PI
        else:
             d = sqrt( d0x*d0x + d0y*d0y ) + sqrt( d1x*d1x + d1y*d1y ) + sqrt( d2x*d2x + d2y*d2y )
             angle = amin/d/d

        # actually not used
        area =  0.5* ( v0.x*v1.y - v1.x*v0.y
                     - v0.x*v2.y + v2.x*v0.y
                     + v1.x*v2.y - v2.x*v1.y  )
        if area < 0.0: return 0,0,0

        return minAngle, angle, area

    def HasBoundingVertices(self,v0,v1,v2):
         return (v0 in self.Vertices[:3] or v1 in self.Vertices[:3] or v2 in self.Vertices[:3])

    def CheckNumber(self, x):
        pass
        # todo: assert x is finite
        #def isnan(x):
        #    return isinstance(x, float) and x != x
        #assert(not isnan(x))

    def ArcTan2(self, x, y):
        self.CheckNumber(x)
        self.CheckNumber(y)
        return atan2(x,y)

    def GetAngle(self, a1, a0):
         d = a1 - a0
         self.CheckNumber(a0)
         self.CheckNumber(a1)
         while d >   tmC_PI: d -= tmC_PIx2
         while d <= -tmC_PI: d += tmC_PIx2
         return d

    def GetSegment(self,v0,v1):
         for i in range(self.segmentCount):
              x0, x1 = self.Segments[i].v
              if (v0==x0 and v1==x1) or (v0==x1 and v1==x0):
                  return self.Segments[i]
         return None

    def GetAdjacentEdges(self, e, t) :
         assert( (e==t.e[0])or(e==t.e[1])or(e==t.e[2]) )

         if      (e==t.e[0]): return (t.e[1], t.e[2], t.v[2])
         elif (e==t.e[1]):    return (t.e[2], t.e[0], t.v[0])
         elif (e==t.e[2]):    return (t.e[0], t.e[1], t.v[1])

    def IsOppositeVertex(self, v0, v1, v2):
         return ( (v1 in self.Vertices[:self.inputVertexCount] )
             and (  self.GetSegment(v0, v1) != None      )
             and (  self.GetSegment(v1, v2) != None      )  )

    def FixEdge(self, e, t0, t1):
         assert( (e.t[0]==t0) or (e.t[1]==t0) )
         
         if   e.t[0]==t0:  e.t[0] = t1
         elif e.t[1]==t0:  e.t[1] = t1 

    def InsertVertexAt(self, v, e):
         t0, t1 = e.t
         v0, v2 = e.v
         e2, e3, v3 = self.GetAdjacentEdges(e, t0)
         e0, e1, v1 = self.GetAdjacentEdges(e, t1)
         
         t2 = self.AddTriangle()
         t3 = self.AddTriangle()

         f0 = self.AddEdge()
         f1 = self.AddEdge()
         f2 = self.AddEdge()
         
         i0     = t0.inside
         i1     = t1.inside
         locked = e.locked
         
         self.SetTriangle( t0, v3, v0, v, e3, e, f2)
         self.SetTriangle( t1, v0, v1, v, e0, f0, e)
         self.SetTriangle( t2, v1, v2, v, e1, f1, f0)
         self.SetTriangle( t3, v2, v3, v, e2, f2, f1)
         
         self.SetEdge(e, v0, v, t0, t1)
         self.SetEdge(f0, v1, v, t1, t2)
         self.SetEdge(f1, v2, v, t2, t3)
         self.SetEdge(f2, v3, v, t3, t0)
         
         self.FixEdge(e1, t1, t2)
         self.FixEdge(e2, t0, t3)
         
         t0.inside = i0
         t1.inside = i1
         t2.inside = i1
         t3.inside = i0
         
         e.locked  = locked
         f1.locked = locked
         
         if i0:
              self.CheckEdge( e2)
              self.CheckEdge( e3)
         if i1:
              self.CheckEdge( e0)
              self.CheckEdge( e1)

         return True

    def InsertVertex(self, v):
        t0 = self.FindVertex(v)
        if t0 == None:
            return False
        
        for i in range(3):
             v0 = t0.v[i]
             if i == 2: v1 = t0.v[0]
             else:      v1 = t0.v[i+1]
             if self.GetVertexPosition(v0, v1, v)==0.0:
                  return self.InsertVertexAt( v, t0.e[i] )
        
        v0, v1, v2 = t0.v
        e0, e1, e2 = t0.e
        
        t1 = self.AddTriangle()
        t2 = self.AddTriangle()
        f0 = self.AddEdge()
        f1 = self.AddEdge()
        f2 = self.AddEdge()
        
        self.SetTriangle( t0, v0, v1, v, e0, f1, f0)
        self.SetTriangle( t1, v1, v2, v, e1, f2, f1)
        self.SetTriangle( t2, v2, v0, v, e2, f0, f2)
        
        self.SetEdge(f0, v0, v, t2, t0)
        self.SetEdge(f1, v1, v, t0, t1)
        self.SetEdge(f2, v2, v, t1, t2)
        
        self.FixEdge(e1, t0, t1)
        self.FixEdge(e2, t0, t2)
        
        self.CheckEdge(e0)
        self.CheckEdge(e1)
        self.CheckEdge(e2)
        return True

    def CheckEdge(self, e):
        if e.locked: return False
        t0, t1 = e.t
        assert( t0.inside==t1.inside )

        v0, v2 = e.v
        e2, e3, v3 = self.GetAdjacentEdges(e, t0)
        e0, e1, v1 = self.GetAdjacentEdges(e, t1)
        if self.GetVertexPosition( v1, v3, v2)>=0.0 or self.GetVertexPosition( v1, v3, v0)<=0.0:
            return False

        cCount = 0
        if self.HasBoundingVertices( v0, v2, v3): cCount+=1
        if self.HasBoundingVertices( v2, v0, v1): cCount+=1
        a0 = t0.minAngle
        a1 = t1.minAngle
        cAngle = min(a0, a1) 

        pCount = 0
        if self.HasBoundingVertices( v1, v3, v0): pCount+=1
        if self.HasBoundingVertices( v3, v1, v2): pCount+=1

        a0, q0, s = self.SetTriangleData( v1, v3, v0)
        a1, q1, s = self.SetTriangleData( v3, v1, v2)
        pAngle = min(a0, a1)

        if pCount<cCount or pAngle>cAngle:
            self.SetTriangle( t0, v1, v3, v0, e, e3, e0)
            self.SetTriangle( t1, v3, v1, v2, e, e1, e2)
            
            self.SetEdge( e, v1, v3, t0, t1)
            self.FixEdge( e0, t1, t0)
            self.FixEdge( e2, t0, t1)
            
            self.CheckEdge( e0)
            self.CheckEdge( e1)
            self.CheckEdge( e2)
            self.CheckEdge( e3)
            return True 
        return False 

    def GetClosestVertex(self, x, y):
        dmin=0.0
        v=None
        
        for i in range(self.vertexCount):
            dx = self.Vertices[i].x - x
            dy = self.Vertices[i].y - y
            d2 = dx*dx + dy*dy
            if i==0 or d2<dmin :
                dmin    = d2
                v = self.Vertices[i]
        return(v)

    def MarkInsideTriangles(self, nonconvex):
         rtn=tmE_OK
         
         if nonconvex:
              self.DeleteTriangle(self.Triangles[1])
              for i in range(self.holeCount):
                   t = self.FindVertex(  self.Holes[i] )
                   if ( t==None ): rtn = tmE_HOLES
                   else:           self.DeleteTriangle(t)
         else:
              for i in range(self.triangleCount):
                   self.Triangles[i].inside = (self.Triangles[i].v[0] in self.Vertices[3:] and
                                               self.Triangles[i].v[1] in self.Vertices[3:] and
                                               self.Triangles[i].v[2] in self.Vertices[3:])
         return rtn

    def DeleteBadTriangles(self):
         tBad=None
         vc=tmVertex()
         
         while self.vertexCount < self.maxVertexCount:
              angle = tmC_BIGNUMBER
              
              for i in range(self.triangleCount):
                   t = self.Triangles[i]
                   if t.inside and t.angle < angle:
                       angle  = t.angle
                       tBad   = t
              
              if (self.options & tmO_MINIMALGRID) and (angle>=self.gradingLowerAngle):
                  return
              
              self.CircumCenter(vc, tBad)
              
              isInside = False
              for i in range(self.segmentCount):
                   if self.ContainsVertex(self.Segments[i].v[0], self.Segments[i].v[1], vc):
                       isInside = self.SplitSegment( self.Segments[i])
              if not isInside:
                   v  = self.AddVertex()
                   if not v: return
                   v.x, v.y = vc.x, vc.y
                   self.InsertVertex( v)

    def FindVertex(self, v):
         # initialize
         t = self.lastTriangle
         if t==None: t = self.Triangles[1]
         # search 
         repeat = True
         while repeat:
             repeat = False
             for i in range(3):
                  v0 = t.v[i]
                  if i==2: v1 = t.v[0]
                  else:    v1 = t.v[i+1]
                  if self.GetVertexPosition(v0, v1, v) < 0.0:
                       e = t.e[i]
                       if   e.t[0]==t: t = e.t[1] 
                       elif e.t[1]==t: t = e.t[0]
                       else: assert( False )
                       repeat = True
                       break
         # found
         self.lastTriangle = t

         if t.inside:
             return t
         else:
             return None

    def ContainsVertex(self, v0, v1, v):
         cx = 0.5 * (v0.x + v1.x)
         cy = 0.5 * (v0.y + v1.y)
         dx = v1.x - cx
         dy = v1.y - cy
         r2 = dx*dx + dy*dy
         dx = v.x - cx
         dy = v.y - cy
         d2 = dx*dx + dy*dy
         return d2 < r2

    def GetVertexPosition(self, a, b, c):
         if c in self.Vertices[:3]:
              d1 = (b.x - a.x)*(c.y - a.y)
              d2 = (b.y - a.y)*(c.x - a.x)
         else:
              d1 = (a.x - c.x)*(b.y - c.y)
              d2 = (a.y - c.y)*(b.x - c.x)
         return d1-d2

    def GetSplitPosition(self, v, v0, v1):
        vt = tmVertex()
        if v1 in self.Vertices[:self.inputVertexCount]:
            v0, v1 = v1, v0

        if v0 in self.Vertices[:self.inputVertexCount]:
            dx = v1.x - v0.x
            dy = v1.y - v0.y
            d  = sqrt(dx*dx + dy*dy)
            # 1) p41
            f  = pow(2.0, floor(tmC_SQRT2 * log(0.5*d) + 0.5) )/d
            v.x   = v0.x + f*dx
            v.y   = v0.y + f*dy
        else :
            v.x = 0.5*(v0.x + v1.x)
            v.y = 0.5*(v0.y + v1.y)

    def SplitSegment(self, s):
         e = self.GetEdge( s.v[0], s.v[1])
         assert(e!=None)
         
         v0 = s.v[0]
         v1 = s.v[1]

         if self.SameVertex(v0,v1): return False

         v = self.AddVertex()
         if not v: return False
         
         t = self.AddSegment()
         self.SetSegment(s, v0, v)
         self.SetSegment(t, v,  v1)
         
         self.GetSplitPosition( v, v0, v1)
         self.InsertVertexAt( v, e)
         return True

    def InsertSegments(self):
        inserting = True

        while inserting:
            inserting = False
            for i in range(self.segmentCount):
                s  = self.Segments[i]
                v0 = s.v[0]
                v1 = s.v[1]
                
                e = self.GetEdge(v0, v1)
                if not e:
                     v = self.AddVertex()
                     if not v: return
                     t = self.AddSegment()
                     self.SetSegment(s, v0, v)
                     self.SetSegment(t, v, v1)
                     
                     self.GetSplitPosition( v, v0, v1)
                     inserting = self.InsertVertex( v)

                elif (self.ContainsVertex(e.v[0], e.v[1], self.GetOppositeVertex(e, e.t[0])) or
                      self.ContainsVertex(e.v[0], e.v[1], self.GetOppositeVertex(e, e.t[1]))):
                     inserting = self.SplitSegment(s)
  
            if self.vertexCount==self.maxVertexCount:
                self.haveEnoughVertices = False
                return
        
        self.haveEnoughVertices = True

    def SetSegment(self,s,v0,v1):
        s.v[0], s.v[1] = v0, v1

    def ConvexHull(self):
        for i in range(self.triangleCount):
            # Check all combinations
            for j in range(3):
                if j == 0: i0, i1, i2 = 0, 1, 2
                elif j==1: i0, i1, i2 = 1, 2, 0
                elif j==2: i0, i1, i2 = 2, 0, 1

                if (self.Triangles[i].v[i0] in self.Vertices[3:] and
                    self.Triangles[i].v[i1] in self.Vertices[3:] and
                    self.Triangles[i].v[i2] in self.Vertices[:3] and
                    self.GetSegment(self.Triangles[i].v[i0], self.Triangles[i].v[i1])==None):
                       s = self.AddSegment()
                       self.SetSegment(s, self.Triangles[i].v[i0], self.Triangles[i].v[i1])
        
    def Reset(self):
        self.Vertices         = []
        self.Edges            = []
        self.Triangles        = []
        self.Segments         = []

        self.vertexCount      = 0
        self.inputVertexCount = 0
        self.edgeCount        = 0
        self.triangleCount    = 0
        self.segmentCount     = 0
        self.holeCount        = 0
        self.insideTriangleCount = 0

    def PolygonCenter(self, v, n, from_=0):
        vc = tmVertex( (v[from_].x, v[from_].y) )
        for i in range(from_+1, n):
            vc.x += v[i].x
            vc.y += v[i].y
        vc.x /= n-from_
        vc.y /= n-from_
        return vc 

    def GetErrorMessage(self, errId):
        if errId in tmErrorMessages:
            return tmErrorMessages[errId]
        else:
            return "unknown error occurred"

    def WriteInput(self, seg):
        pass

    def PrintData(self):
         print "Options    : %d" % (self.options)
         print "MinAngle   : %G" % (self.gradingLowerAngle)
         print "Max V/E/T/S: %d %d %d %d" % (self.maxVertexCount,self.maxEdgeCount,self.maxTriangleCount,self.maxSegmentCount)
         print "    actual : %d %d %d %d %d" % (self.vertexCount,self.edgeCount, self.triangleCount, self.segmentCount, self.holeCount)
         print "self.Vertices   : %d" % (self.vertexCount)
         print "self.Segments   : %d" % (self.segmentCount)
         print "self.Triangles  : %d (total: %d)" % (self.insideTriangleCount,self.triangleCount)

    def PrintTriangles():
        for t in self.Triangles:
            print "%04d;%6.2f;%6.2f;%6.2f;%6.2f;%6.2f;%6.2f;%d;%6.2f;%6.2f\n" % (i,
                   t.v[0].x,t.v[0].y,t.v[1].x,t.v[1].y,
                   t.v[2].x,t.v[2].y,t.inside,t.minAngle,t.angle)

def main():
    rtn = 0
    # the geometry-boundary to mesh, points in length units.
    # a ring
    #
    # node points
    nodes = ( ( 5.00,	 0.00), #  1 outer boundary
              ( 3.54,	 3.54), #  2
              ( 0.00,	 5.00), #  3
              (-3.54,	 3.54), #  4
              (-5.00,	 0.00), #  5
              (-3.54,	-3.54), #  6
              ( 0.00,	-5.00), #  7
              ( 3.54,	-3.54), #  8

              ( 2.00,	 0.00), #  9 inner boundary
              ( 1.41,	 1.41), # 10
              ( 0.00,	 2.00), # 11
              (-1.41,	 1.41), # 12
              (-2.00,	 0.00), # 13
              (-1.41,	-1.41), # 14
              ( 0.00,	-2.00), # 15
              ( 1.41,	-1.41)) # 16

    nodes = [tmVertex(node) for node in nodes]

    holes = [ tmVertex( (0.0, 0.0) ) ]

    # center hole boundary 
    segs = (  ( 9, 10), # point indices (see nodes[]) starting at 1
              (10, 11),
              (11, 12),
              (12, 13),
              (13, 14),
              (14, 15),
              (15, 16),
              (16,  9))

    segs = [tmSegmentId(seg) for seg in segs]

    # instead of nodes indices
    segXY =  (( 2.00,    0.00), #  inner boundary
              ( 1.41,    1.41),
              ( 0.00,    2.00),
              (-1.41,    1.41),
              (-2.00,    0.00),
              (-1.41,   -1.41),
              ( 0.00,   -2.00),
              ( 1.41,   -1.41))

    segXY = [tmVertex(seg) for seg in segXY]

    # go
    md  = TriangleMesh()

    # 1. possibility
    rtn = md.Mesh( nodes, segs, holes)
    vc  = md.PolygonCenter(nodes, 16, 8)
    print " PolygonCenter: [%G %G]" % (vc.x,vc.y)
    print " %s [%d]" % (md.GetErrorMessage(rtn), rtn)

    md.PrintData()
    md.Reset()

    exit(0)
    # not working in python yet...
    # 2. possibility
    rtn = md.Mesh( nodes[:8], segXY, holes)
    print " %s [%d]" % (md.GetErrorMessage(rtn), rtn)
    md.PrintData()

if __name__=="__main__":
     main()
