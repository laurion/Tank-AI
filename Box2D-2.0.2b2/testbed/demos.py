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

import os
import pygame
import Box2D as box2d
from pygame.locals import *
from settings import fwSettings
from pgu import gui
from test_main import fwDebugDraw

from distutils import sysconfig

global width, height
width, height = 640, 480

def main():
    # Initialize pygame
    screen = pygame.display.set_mode((width,height))

    caption= "Python Box2D Testbed Demos"
    pygame.display.set_caption(caption)

    # Initialize the GUI
    theme = gui.Theme("default")
    app = gui.Desktop(theme=theme)

    app.connect(gui.QUIT,app.quit,None)

    main = gui.Container(width=width, height=height)

    # Label at the top left
    main.add(gui.Label("Box2D Testbed Demos", cls="h1"), 20, 20)

    # The dimensions of the list of the demos
    list_size= (width / 2, height / 2)
    list_pos = (width/2 - list_size[0]/2, height/2 - list_size[1]/2)

    # Create the actual widget
    demolist = gui.List(width=list_size[0], height=list_size[1])
    main.add(demolist, list_pos[0], list_pos[1])

    # Add all the demos found in the directory to the list
    add_demos(demolist)

    
    buttonw = (list_size[0]/2-20)        # width of the buttons
    bottom = list_pos[1]+list_size[1]+20 # the y-location of the bottom of the list

    # Create a Run button, calling run_demo on click
    b = gui.Button("Run", width=buttonw)
    main.add(b, list_pos[0], bottom)
    b.connect(gui.CLICK, run_demo, demolist)

    # Create a Quit button
    b = gui.Button("Quit", width=buttonw)
    main.add(b, list_pos[0]+buttonw+30, bottom)
    b.connect(gui.CLICK, lambda x:pygame.event.post(pygame.event.Event(pygame.QUIT)), None)

    # box2d initialization
    z=10 #scale (10 pixels/physics unit)

    # Since we're taking the debug draw implementation from the testbed, it requires a bit
    # of ugliness to set it up properly.
    renderer = fwDebugDraw()
    renderer.surface = screen
    renderer.viewZoom=z
    renderer.viewCenter=box2d.b2Vec2(0,0)
    renderer.width, renderer.height = width, height
    renderer.viewOffset = renderer.viewCenter - box2d.b2Vec2(width, height)/2
    renderer.SetFlags(box2d.b2DebugDraw.e_shapeBit) # we only want shapes to be drawn
    renderer.DrawSolidPolygon = lambda *args: 0 # make it not draw the polygons!
    renderer.DrawPolygon = lambda *args: 0      #

    # Create the world
    worldAABB=box2d.b2AABB()
    worldAABB.lowerBound = (-100.0, -100.0)
    worldAABB.upperBound = ( 100.0, 100.0)
    gravity = (0.0, -10.0)

    world = box2d.b2World(worldAABB, gravity, True)
    world.SetDebugDraw(renderer)

    # Create the ground
    bd = box2d.b2BodyDef()
    bd.position = (0.0, 0.0)
    ground = world.CreateBody(bd)
    
    # The borders of the screen
    sd = box2d.b2PolygonDef()
    sd.SetAsBox(1, height/z, (-width/(2*z)-1, 0), 0)
    ground.CreateShape(sd)
    sd.SetAsBox(1, height/z, (width/(2*z)+1, 0), 0)
    ground.CreateShape(sd)
    sd.SetAsBox(width/z, 1, (0,-height/(2*z)-1), 0)
    ground.CreateShape(sd)
    sd.SetAsBox(width/z, 1, (0,height/(2*z)+1), 0)
    ground.CreateShape(sd)

    # The shape for the file list
    sd.SetAsBox(list_size[0]/(2*z), list_size[1]/(2*z))
    ground.CreateShape(sd)

    # Create a few balls to bounce around the screen
    for i in range(10):
        bd = box2d.b2BodyDef()
        bd.allowSleep = True
        bd.position = (box2d.b2Random(-width/(2*z), width/(2*z)), box2d.b2Random(-height/(2*z), height/(2*z)))
        bd.isBullet = True
        bomb = world.CreateBody(bd)
        bomb.SetLinearVelocity(-5.0 * bd.position)

        sd = box2d.b2CircleDef()
        sd.radius = 1
        sd.density = 1.0
        sd.restitution = 0.7
        bomb.CreateShape(sd)
        
        bomb.SetMassFromShapes()

    app.init(main)
    main_loop(world, screen, demolist, app)

def get_shapes(world):
    for body in world.bodyList:
        shape = body.GetShapeList()
        while shape:
            yield (body, shape)
            shape=shape.GetNext()

def update_shapes(world):
    # Check to see if any objects are sleeping or coming to a stop. 
    # Then give them a little push.
    for body in world.bodyList:
        v = body.GetLinearVelocity()
        if body.IsSleeping() or v.LengthSquared() < 0.2:
            i = body.GetWorldVector((box2d.b2Random(-200,200), box2d.b2Random(-200,200)))
            p = body.GetWorldPoint((0.0, 0.0))
            body.ApplyImpulse(i, p)
       
def main_loop(world, screen, demolist, app):
    # Create a surface to draw the GUI onto
    app_surface = pygame.Surface((width,height), SWSURFACE)
    hz = 60.0
    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                return
            elif event.type == KEYDOWN:
                if event.key == K_RETURN:
                    run_demo(demolist)
            elif event.type == MOUSEBUTTONDOWN:
                if event.button ==3:
                    run_demo(demolist)
                elif event.button ==4:
                    demolist.set_vertical_scroll(demolist.vscrollbar.value-100)
                elif event.button ==5:
                    demolist.set_vertical_scroll(demolist.vscrollbar.value+100)

            app.event(event)

        # Tell the GUI to update itself, then blit it to the screen
        app.update(app_surface) # pygame 1.8.1/pgu problem? 1.8.0 works fine
        screen.blit(app_surface, (0,0))

        # Wake up non/slow-moving shapes
        update_shapes(world)

        # Step the world 
        # (calls the debugdraw functions, so this is where the balls are drawn)
        world.Step(1/hz, 10, 8)

        clock.tick(hz)        
        #fps = clock.get_fps()
        pygame.display.flip()

def add_demos(demolist):
    # I don't feel like maintaining a list of demos.
    # So just glob to see what test_*.py files are in the current
    # directory and add them to the demo list.
    import glob

    ignore_list=("main")

    for f in glob.glob("test_*.py"):
        name = f[5:-3]
        if name.lower() in ignore_list:
            continue
        demolist.add(name,value=f)

def run_demo(demolist):
    # Run the currently selected demo (the widget itself is passed as an argument)
    if demolist.value == None: return

    print "Running: ", demolist.value
    from sys import executable as python_interpreter
    from platform import system as sys_platform

    if sys_platform() == "Windows":
        # Just in case we're in some directory with a space in it, use quotes
        # around the path.
        cmd = '"%s" -OO %s' % (python_interpreter, demolist.value)
    else:
        cmd = "%s -OO %s" % (python_interpreter, demolist.value)

    print "-> %s" % cmd

    ret = os.system(cmd)

    if ret == 10 or ret/256 == 10: # user hit reload (somehow the retval on linux is *256)
        run_demo(demolist) 

if __name__=="__main__":
    main()
