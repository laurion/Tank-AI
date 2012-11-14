#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/python
#
# Original C++ version by Daid 
#  http://www.box2d.org/forum/viewtopic.php?f=3&t=1473
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

from test_main import *
class BoxCutter (Framework):
    name="BoxCutter"
    laserBody=None
    _pickle_vars=['laserBody']
    def __init__(self):
        super(BoxCutter, self).__init__() 

        self.world.GetGroundBody().SetUserData("ground")
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -10.0)
        bd.userData = "ground1"
        ground = self.world.CreateBody(bd)
        
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        ground.CreateShape(sd)
    
        bd=box2d.b2BodyDef()
        bd.position = (0.0, 50.0)
        bd.userData = "ground2"
        ground = self.world.CreateBody(bd)
        
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(50.0, 10.0)
        ground.CreateShape(sd)
    
        bd=box2d.b2BodyDef()
        bd.position = (0.0, 1.0)
        bd.userData = "laser"
        self.laserBody = self.world.CreateBody(bd)
        
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(5.0, 1.0)
        sd.density = 4.0
        self.laserBody.CreateShape(sd)
        self.laserBody.SetMassFromShapes()
    
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(3.0, 3.0)
        sd.density = 5.0
        
        bd=box2d.b2BodyDef()
        bd.userData = 1
        bd.position = (0.0, 8.0)
        body1 = self.world.CreateBody(bd)
        body1.CreateShape(sd)
        body1.SetMassFromShapes()
    
        sd=box2d.b2PolygonDef()
        sd.SetAsBox(3.0, 3.0)
        sd.density = 5.0
        
        bd=box2d.b2BodyDef()
        bd.userData = 1
        bd.position = (0.0, 8.0)
        body1 = self.world.CreateBody(bd)
        body1.CreateShape(sd)
        body1.SetMassFromShapes()

    # Split a shape through a segment
    # Returns:
    #  False - Error on split
    #  True  - Normal result is two new shape definitions.    
    def SplitShape(self, shape, segment, splitSize, newPolygon):
        b = shape.GetBody()
        xf = b.GetXForm()
        hit,lambda_,normal=shape.TestSegment(xf, segment, 1.0)
        if hit != box2d.e_hitCollide:
            return False
        entryPoint = (1-lambda_)*segment.p1+lambda_*segment.p2
        
        reverseSegment=box2d.b2Segment()
        reverseSegment.p1=segment.p2
        reverseSegment.p2=segment.p1
        
        hit,lambda_,normal=shape.TestSegment(xf, reverseSegment, 1.0)
        if hit != box2d.e_hitCollide:
            return False

        exitPoint = (1-lambda_)*reverseSegment.p1+lambda_*reverseSegment.p2
        
        localEntryPoint = b.GetLocalPoint(entryPoint)
        localExitPoint = b.GetLocalPoint(exitPoint)
        vertices = shape.getVertices_b2Vec2()
        cutAdded = [-1,-1]
        last = -1
        for i in range(shape.GetVertexCount()):
            #Find out if this vertex is on the old or new shape.
            if box2d.b2Dot(box2d.b2Cross(localExitPoint-localEntryPoint, 1), vertices[i]-localEntryPoint) > 0:
                n = 0
            else:
                n = 1
            if last != n:
                #If we switch from one shape to the other add the cut vertices.
                if last == 0:
                    assert(cutAdded[0]==-1)
                    cutAdded[0] = newPolygon[last].vertexCount
                    newPolygon[last].setVertex(newPolygon[last].vertexCount, localExitPoint)
                    newPolygon[last].vertexCount+=1
                    newPolygon[last].setVertex(newPolygon[last].vertexCount, localEntryPoint)
                    newPolygon[last].vertexCount+=1
                elif last == 1:
                    assert(cutAdded[last]==-1)
                    cutAdded[last] = newPolygon[last].vertexCount
                    newPolygon[last].setVertex(newPolygon[last].vertexCount, localEntryPoint)
                    newPolygon[last].vertexCount+=1
                    newPolygon[last].setVertex(newPolygon[last].vertexCount, localExitPoint)
                    newPolygon[last].vertexCount+=1
            newPolygon[n].setVertex(newPolygon[n].vertexCount, vertices[i])
            newPolygon[n].vertexCount+=1
            last = n
        
        #Add the cut in case it has not been added yet.
        if cutAdded[0]==-1:
            cutAdded[0] = newPolygon[0].vertexCount
            newPolygon[0].setVertex(newPolygon[0].vertexCount, localExitPoint)
            newPolygon[0].vertexCount+=1
            newPolygon[0].setVertex(newPolygon[0].vertexCount, localEntryPoint)
            newPolygon[0].vertexCount+=1
        if cutAdded[1]==-1:
            cutAdded[1] = newPolygon[1].vertexCount
            newPolygon[1].setVertex(newPolygon[1].vertexCount, localEntryPoint)
            newPolygon[1].vertexCount+=1
            newPolygon[1].setVertex(newPolygon[1].vertexCount, localExitPoint)
            newPolygon[1].vertexCount+=1
        
        # Cut based on the split size
        for n in range(2):
            if cutAdded[n] > 0:
                offset = newPolygon[n].getVertex(cutAdded[n]-1) - newPolygon[n].getVertex(cutAdded[n])
            else:
                offset = newPolygon[n].getVertex(newPolygon[n].vertexCount-1) - newPolygon[n].getVertex(0)

            offset.Normalize()
            
            newPolygon[n].setVertex(cutAdded[n], newPolygon[n].getVertex(cutAdded[n]) + splitSize * offset)
            
            if cutAdded[n] < newPolygon[n].vertexCount-2:
                offset = newPolygon[n].getVertex(cutAdded[n]+2) - newPolygon[n].getVertex(cutAdded[n]+1)
            else:
                offset = newPolygon[n].getVertex(0) - newPolygon[n].getVertex(newPolygon[n].vertexCount-1)
            offset.Normalize()
            
            newPolygon[n].setVertex(cutAdded[n]+1, newPolygon[n].getVertex(cutAdded[n]+1) + splitSize * offset)
        
        #Check if the new shapes are not too tiny.
        for n in range(2):
            for i in range(newPolygon[n].vertexCount):
                for j in range(newPolygon[n].vertexCount):
                    if i != j and (newPolygon[n].getVertex(i) - newPolygon[n].getVertex(j)).Length() < 0.1:
                        return False
        # Make sure the shapes are valid before creation
        try:
            for n in range(2):
                box2d.b2CheckPolygonDef(newPolygon[n])
        except ValueError, s:
            print "Created bad shape:", s
            return False

        return True
    
    def Cut(self):
        segmentLength = 30.0
        
        segment=box2d.b2Segment()
        
        laserStart=(5.0-0.1,0.0)
        laserDir  =(segmentLength,0.0)
        
        segment.p1 = self.laserBody.GetWorldPoint(laserStart)
        segment.p2 = segment.p1 + self.laserBody.GetWorldVector(laserDir)
        
        max_shapes = 64
        count, shapes = self.world.Raycast(segment, max_shapes, False, None)
        for i in range(count):
            #Make sure it's a polygon, we cannot cut circles.
            if shapes[i].GetType() != box2d.e_polygonShape:
                continue

            polyShape = shapes[i]
            
            b = polyShape.GetBody()
            #Custom check to make sure we don't cut stuff we don't want to cut.
            if b.GetUserData() != 1:
                continue #return if we cannot pass trough uncutable shapes.
            
            pd=[box2d.b2PolygonDef(),box2d.b2PolygonDef()]
            pd[0].density = 5.0
            pd[1].density = 5.0
            
            if self.SplitShape(polyShape, segment, 0.1, pd):
                b.DestroyShape(shapes[i])
                b.CreateShape(pd[0])
                b.SetMassFromShapes()
                b.WakeUp()
                
                bd=box2d.b2BodyDef()
                bd.userData = 1
                bd.position = b.GetPosition()
                bd.angle = b.GetAngle()
                newBody = self.world.CreateBody(bd)
                newBody.CreateShape(pd[1])
                newBody.SetMassFromShapes()
                newBody.SetAngularVelocity(b.GetAngularVelocity())
                newBody.SetLinearVelocity(b.GetLinearVelocity())
    
    def Keyboard(self, key):
        if key==K_c:
            self.Cut()
    
    def Step(self, settings):
        super(BoxCutter, self).Step(settings)
    
        if not self.laserBody:
            return

        self.DrawStringCR("Keys: Cut = c")

        segmentLength = 30.0
        
        segment=box2d.b2Segment()
        laserStart=(5.0-0.1,0.0)
        laserDir  =(segmentLength,0.0)
        segment.p1 = self.laserBody.GetWorldPoint(laserStart)
        segment.p2 = segment.p1+self.laserBody.GetWorldVector(laserDir)
        laserColor=box2d.b2Color(1,0,0)

        self.debugDraw.DrawSegment(segment.p1,segment.p2,laserColor)

if __name__=="__main__":
    main(BoxCutter)
