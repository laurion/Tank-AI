#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is a simple example of building and running a simulation
using Box2D. Here we create a large ground box and a small dynamic box.

NOTE:
There is no graphical output for this simple example, only text.
"""
 
from Box2D import *
 
# Define the size of the world. Simulation will still work
# if bodies reach the end of the world, but it will be slower.
worldAABB=b2AABB()
worldAABB.lowerBound = (-100, -100)
worldAABB.upperBound = ( 100,  100)

# Define the gravity vector.
gravity = b2Vec2(0, -10)
 
# Do we want to let bodies sleep?
doSleep = True
 
# Construct a world object, which will hold and simulate the rigid bodies.
world = b2World(worldAABB, gravity, doSleep)

# Define the ground body.
groundBodyDef = b2BodyDef()
groundBodyDef.position = [0, -10]
 
# Call the body factory which allocates memory for the ground body
# from a pool and creates the ground box shape (also from a pool).
# The body is also added to the world.
groundBody = world.CreateBody(groundBodyDef)
 
# Define the ground box shape.
groundShapeDef = b2PolygonDef()
 
# The extents are the half-widths of the box.
groundShapeDef.SetAsBox(50, 10)
 
# Add the ground shape to the ground body.
groundBody.CreateShape(groundShapeDef)
 
# Define the dynamic body. We set its position and call the body factory.
bodyDef = b2BodyDef()
bodyDef.position = (0, 4)
body = world.CreateBody(bodyDef)
 
# Define another box shape for our dynamic body.
shapeDef = b2PolygonDef()
shapeDef.SetAsBox(1, 1)
 
# Set the box density to be non-zero, so it will be dynamic.
shapeDef.density = 1
 
# Override the default friction.
shapeDef.friction = 0.3
 
# Add the shape to the body.
shape=body.CreateShape(shapeDef)
 
# Now tell the dynamic body to compute it's mass properties base on its shape.
body.SetMassFromShapes()
 
# Prepare for simulation. Typically we use a time step of 1/60 of a
# second (60Hz) and 10 velocity/8 position iterations. This provides a 
# high quality simulation in most game scenarios.
timeStep = 1.0 / 60
vel_iters, pos_iters = 10, 8
 
# This is our little game loop.
for i in range(60):
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    world.Step(timeStep, vel_iters, pos_iters)
 
    # Now print the position and angle of the body.
    print body.position, body.angle

