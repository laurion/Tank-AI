#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
# Python version Copyright (c) 2008 kne / sirkne at gmail dot com
# Cairo backend Copyright (c) 2009 Giorgos Giagas / giorgosg at gmail dot com
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

from __future__ import division
import math
import time
import operator
import itertools

import pygtk
pygtk.require('2.0')
import gtk, gobject, cairo

import Box2D as box2d
from settings import fwSettings
import console

from pygame_keycodes import *

# Use psyco if available
try:
    import psyco
    psyco.full()
except ImportError:
    pass

class FpsCount(object):
    def __init__(self, avg_count):
        self._list = [0 for i in range(avg_count)]
        self.oldest = 0
        self.count = avg_count
    def __call__(self):
        new_time = time.time()
        self._list[self.oldest] = time.time()
        self.oldest = self.oldest + 1 if self.oldest + 1 < self.count else 0
        avg_frame_time = (new_time - self._list[self.oldest]) / self.count
        return 1 / avg_frame_time

class CairoDebugDraw(box2d.b2DebugDraw):
    def __init__(self):
        super(CairoDebugDraw, self).__init__()
        self.scale = 8 
        self.view = (-50.0, 50.0) # the upper left corner in b2world coords.
        self.line_width = 0.04 # in world units
        self.min_line_width = 1.0 # in screen units
        # some colors look bad when inverted
        self.colordict = {(  0,   0, 204): [0.0, 0.0, 0.8],
                          (127, 229, 127): [0.56, 0.22, 0.0]}
        self.antialias = cairo.ANTIALIAS_NONE
        self.font_options = cairo.FontOptions()
        self.font_options.set_antialias(self.antialias)

    def draw_begin(self, drawing_area):
        width, height = drawing_area.window.get_size()

        region = gtk.gdk.region_rectangle((0, 0, width, height))
        drawing_area.window.begin_paint_region(region)
        cr = self.cr = drawing_area.window.cairo_create()
        cr.set_antialias(self.antialias)
        cr.set_font_options(self.font_options)

        cr.rectangle(0, 0, width, height)
        cr.clip()
        cr.save()
        cr.set_source_rgb(1.0, 1.0, 1.0)
        cr.paint()

        cr.scale(self.scale, -self.scale)
        cr.translate(- self.view[0],
                     - self.view[1])
        
        if self.line_width * self.scale >= self.min_line_width:
            line_width = self.line_width
        else:
            line_width = self.min_line_width / self.scale
            
        cr.set_tolerance(0.2)
        cr.set_line_width(line_width)

        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.set_line_cap(cairo.LINE_CAP_ROUND)

    def draw_end(self, drawing_area):
        drawing_area.window.end_paint()

    def screen_to_world(self, pos):
        pos = (pos[0] / self.scale, -pos[1] / self.scale)
        pos = (pos[0] + self.view[0], pos[1] + self.view[1])
        return pos
    def world_to_screen(self, pos):
        pos = (pos[0] * self.scale, -pos[0] * self.scale)
        pos = (pos[0] - (self.view[0]*self.scale),
               pos[1] - (-self.view[0]*self.scale))
        return pos
    
    def convert_color(self, color):
        color = (color.r, color.g, color.b)
        lkup = tuple(int(c*255) for c in color)
        if lkup in self.colordict:
            return self.colordict[lkup]
        else:
            return [1-c for c in color]

    def DrawPolygon(self, vertices, color):
        cr = self.cr
        color = self.convert_color(color)
        cr.set_source_rgb(color[0], color[1], color[2])
        cr.move_to(*vertices[0])
        for v in vertices:
            cr.line_to(*v)
        cr.close_path()
        cr.stroke()
    
    def DrawSolidPolygon(self, vertices, color):
        cr = self.cr
        color = self.convert_color(color)
        cr.set_source_rgba(color[0], color[1], color[2], 0.7)
        cr.move_to(*vertices[0])
        for v in vertices:
            cr.line_to(*v)
        cr.close_path()
        cr.fill_preserve()

        cr.set_source_rgb(color[0], color[1], color[2])
        cr.stroke()

    def DrawCircle(self, center, radius, color):
        cr = self.cr
        color = self.convert_color(color)
        cr.set_source_rgb(color[0], color[1], color[2])
        cr.arc(center[0], center[1], radius, 0, 2*math.pi)
        cr.stroke()

    def DrawSolidCircle(self, center, radius, axis, color):
        cr = self.cr
        color = self.convert_color(color)
        cr.set_source_rgba(color[0], color[1], color[2], 0.7)
        cr.arc(center[0], center[1], radius, 0, 2*math.pi)
        cr.fill_preserve()

        cr.set_source_rgb(color[0], color[1], color[2])
        cr.stroke()

        p = radius * box2d.b2Vec2(axis)
        cr.move_to(center[0], center[1])
        cr.line_to(center[0] + p[0], center[1] + p[1])
        cr.stroke()
               
    def DrawSegment(self, v1, v2, color):
        color = self.convert_color(color)
        cr = self.cr
        cr.set_source_rgba(color[0], color[1], color[2], 0.5)
        cr.move_to(v1[0], v1[1])
        cr.line_to(v2[0], v2[1])
        cr.stroke()

    def DrawPoint(self, p, size, color):
        self.DrawCircle(p, size/self.scale, color)
    
    def DrawXForm(self, xf):
        cr = self.cr
        k_axis_scale = 0.4
        p1 = xf.position
        p2 = p1 + xf.R.col1 * k_axis_scale
        p3 = p1 + xf.R.col2 * k_axis_scale
        cr.set_source_rgba(0,0,1,0.4)
        cr.move_to(p1.x, p1.y)
        cr.line_to(p2.x, p2.y)
        cr.stroke()
        cr.set_source_rgba(1,0,0,0.4)
        cr.move_to(p1.x, p1.y)
        cr.line_to(p3.x, p3.y)
        cr.stroke()

class Framework(object):
    """
    The main testbed framework.
    It handles basically everything:
    * The initialization of pygame, Box2D
    * Contains the main loop
    * Handles all user input.

    You should derive your class from this one to implement your own tests.
    See test_Empty.py or any of the other tests for more information.
    """
    name = "None"

    # Box2D-related
    worldAABB          = None
    points             = []
    world              = None
    bomb               = None
    settings           = fwSettings
    bombSpawning       = False
    bombSpawnPoint     = None
    mouseWorld         = None
    destroyList        = []

    # Box2D-callbacks
    destructionListener= None
    boundaryListener   = None
    contactListener    = None
    debugDraw          = None

    def __init__(self):
        # Box2D Initialization
        self.worldAABB=box2d.b2AABB()
        self.worldAABB.lowerBound = (-200.0, -100.0)
        self.worldAABB.upperBound = ( 200.0, 200.0)
        gravity = (0.0, -10.0)
        doSleep = True

        self.world = box2d.b2World(self.worldAABB, gravity, doSleep)
        
        self.debugDraw = CairoDebugDraw()
        self.world.SetDebugDraw(self.debugDraw)

        settings = fwSettings
        self.flag_info = [ ('draw_shapes', settings.drawShapes,
                            box2d.b2DebugDraw.e_shapeBit),
                           ('draw_joints', settings.drawJoints,
                            box2d.b2DebugDraw.e_jointBit),
                           ('draw_controlers', settings.drawControllers,
                            box2d.b2DebugDraw.e_controllerBit),
                           ('draw_core_shapes', settings.drawCoreShapes,
                            box2d.b2DebugDraw.e_coreShapeBit),
                           ('draw_aabbs', settings.drawAABBs,
                            box2d.b2DebugDraw.e_aabbBit),
                           ('draw_obbs', settings.drawOBBs,
                            box2d.b2DebugDraw.e_obbBit),
                           ('draw_pairs', settings.drawPairs,
                            box2d.b2DebugDraw.e_pairBit),
                           ('draw_center_of_masses', settings.drawCOMs,
                            box2d.b2DebugDraw.e_centerOfMassBit),]

        builder = self.builder = gtk.Builder()
        builder.add_from_file('cairo_main.xml')
        builder.connect_signals(self)

        #builder.get_object('messages_expander').set_expanded(True)
        
        self.drawing_area = builder.get_object('drawing_area')
        self.window = builder.get_object('window')
        self.window.set_title("Python Box2D Cairo Testbed - " + self.name)

        self.window.show_all()

        gobject.timeout_add(int(1/self.settings.hz*1000), self.SimulationLoop,
                            priority=gobject.PRIORITY_LOW)
        # Need to check if this is dropping the overall fps
        #gobject.timeout_add(100, self.Draw,
        #                    priority=gobject.PRIORITY_DEFAULT_IDLE)
        builder.get_object('hertz').set_value(self.settings.hz)
        builder.get_object('position_iterations'
                           ).set_value(self.settings.positionIterations)
        builder.get_object('velocity_iterations'
                           ).set_value(self.settings.velocityIterations)
        builder.get_object('antialiasing'
                           ).set_active(self.debugDraw.antialias ==
                                        cairo.ANTIALIAS_DEFAULT)
        for widget, setting, b2 in self.flag_info:
            builder.get_object(widget).set_active(setting)
        
        self.scrolling = False
        self.hz_changed = False
        self.mouse_joint = None
        self.mouse_coords = (0,0)
        self.text_lines = []
        self.fps = FpsCount(30)

    def on_window_destroy(self, widget):
        gtk.main_quit()
    def on_py_console_clicked(self, widget):
        pyconsole = console.gtk_console(self.window,
                                        {'world':self.world, 'framework':self},
                                        'python interactive console')
        self.window.set_data('PythonConsole', pyconsole)
        

    def on_step_button_clicked(self, widget):
        self.settings.pause = True
        self.SimulationLoop(True)
    def on_play_button_clicked(self, widget):
        self.settings.pause = False
    def on_pause_button_clicked(self, widget):
        self.settings.pause = True

    def on_draw_toggled(self, widget):
        active_flags = [flag[2] for flag in self.flag_info
                        if self.builder.get_object(flag[0]).get_active()] + [0]
        self.debugDraw.SetFlags(reduce(operator.or_, active_flags))
    def on_antialiasing_toggled(self, widget):
        antialiasing = self.builder.get_object('antialiasing').get_active()
        self.debugDraw.antialias = cairo.ANTIALIAS_DEFAULT if antialiasing\
                                   else cairo.ANTIALIAS_NONE
        self.debugDraw.font_options.set_antialias(self.debugDraw.antialias)

    def on_velocity_iterations_value_changed(self, widget):
        self.settings.velocityIterations = widget.get_value_as_int()
    def on_position_iterations_value_changed(self, widget):
        self.settings.positionIterations = widget.get_value_as_int()
    def on_hertz_value_changed(self, widget):
        self.settings.hz = widget.get_value_as_int()
        self.hz_changed = True

    def on_drawing_area_key_press_event(self, widget, event):
        if event.keyval == K_z:   #zoom in
            self.debugDraw.scale *= 1.1
        elif event.keyval == K_x: #zoom out
            self.debugDraw.scale /= 1.1
        else:
            self.Keyboard(event.keyval)
    def Keyboard(self, keyval):
        pass
    
    def on_drawing_area_button_press_event(self, widget, event):
        widget.grab_focus()
        if event.button == 3:
            self.scrolling = (event.x, event.y)
        elif event.button == 1:
            self.create_mouse_joint(event.x, event.y)
    def on_drawing_area_button_release_event(self, widget, event):
        if event.button == 3:
            self.scrolling = False
        elif event.button == 1 and self.mouse_joint:
            self.world.DestroyJoint(self.mouse_joint)
            self.mouse_joint = None
    def on_drawing_area_motion_notify_event(self, widget, event):
        if self.scrolling !=False:
            distance = (self.scrolling[0] - event.x,
                        self.scrolling[1] - event.y)
            distance = (distance[0] / self.debugDraw.scale,
                        - distance[1] / self.debugDraw.scale)
            self.debugDraw.view = (self.debugDraw.view[0] + distance[0],
                                   self.debugDraw.view[1] + distance[1])
            self.scrolling = (event.x, event.y)
        if self.mouse_joint:
            world_coords = self.debugDraw.screen_to_world((event.x, event.y))
            self.mouse_joint.SetTarget(world_coords)
        self.mouse_coords = (event.x, event.y)

    def on_drawing_area_scroll_event(self, widget, event):
        if event.direction not in (gtk.gdk.SCROLL_UP, gtk.gdk.SCROLL_DOWN):
            return
        # we keep the center of the screen in the same place as we zoom.
        da_res = self.drawing_area.window.get_size()
        s = self.debugDraw.scale
        view_w = (da_res[0] / (s*2) , - da_res[1] / (s*2))
        world_center = (box2d.b2Vec2(view_w) +
                        box2d.b2Vec2(self.debugDraw.view))

        if event.direction == gtk.gdk.SCROLL_UP:
            self.debugDraw.scale *= 1.1
        elif event.direction == gtk.gdk.SCROLL_DOWN:
            self.debugDraw.scale /= 1.1

        s = self.debugDraw.scale
        view_w = (da_res[0] / (s*2), - da_res[1] / (s*2))
        new_view = world_center - view_w
        self.debugDraw.view = (new_view.x, new_view.y)

    def create_mouse_joint(self, x, y):
        """ create a mouse joint to the body on world coordinates x, y """
        if self.mouse_joint:
            return
        world_coord = box2d.b2Vec2(self.debugDraw.screen_to_world((x, y)))
        aabb = box2d.b2AABB()
        aabb.lowerBound = world_coord - (0.001, 0.001)
        aabb.upperBound = world_coord + (0.001, 0.001)
        max_count = 10
        count, shapes = self.world.Query(aabb, max_count)

        body = (body for shape, body in ((shape, shape.GetBody())
                                         for shape in shapes)
                if not body.IsStatic()
                and body.GetMass() > 0.0
                and shape.TestPoint(body.GetXForm(), world_coord))
        body = list(itertools.islice(body, 1))
        body = body[0] if len(body) == 1 else None
        if body:
            md = box2d.b2MouseJointDef()
            md.body1 = self.world.GetGroundBody()
            md.body2 = body
            md.target = world_coord
            md.maxForce = 1000 * body.GetMass()
            self.mouse_joint = self.world.CreateJoint(md)
            body.WakeUp()

    def run(self):
        """
        Main loop.
        """
        gtk.main()

    def Draw(self, settings=None):
        #print 'idle redrawing'
        self.world.Step(0, 0, 0)

    def draw_extra(self):
        if self.mouse_joint:
            body = self.mouse_joint.GetBody2()
            p1 = body.GetWorldPoint(self.mouse_joint.localAnchor)
            p2 = self.mouse_joint.target
            self.debugDraw.DrawSegment(p1, p2, box2d.b2Color(1, 1, 1))
        
        #message_label = self.builder.get_object('messages_label')
        #if self.builder.get_object('messages_expander').get_expanded() == True:
        #    self.builder.get_object('messages_label'
        #                            ).set_text('\n'.join(self.text_lines))

        cr = self.debugDraw.cr
        cr.restore()
        cr.scale(1.5,1.5)
        pos = 20
        for line in self.text_lines:
            cr.move_to(10, pos)
            cr.show_text(line)
            pos += 10
        
    def SimulationLoop(self, ignore_pause=False):
        self.debugDraw.draw_begin(self.drawing_area)
        if ignore_pause or (self.settings.pause == False):
            self.text_lines = []
            self.DrawStringCR("Fps: %.1f" % (self.fps(),))
            self.Step(self.settings)
        else:
            self.Draw()

        self.draw_extra()
        self.debugDraw.draw_end(self.drawing_area)

        if self.hz_changed:
            gobject.timeout_add(int(1/self.settings.hz*1000),
                                self.SimulationLoop,
                                priority=gobject.PRIORITY_LOW)
            self.hz_changed = False
            return False
        else:
            return True

    def Step(self, settings):
        """
        The main physics step.

        Takes care of physics drawing
        (callbacks are executed after the world.Step() )
        """
        timestep = 1/settings.hz
        self.world.Step(timestep,
                        settings.velocityIterations,
                        settings.positionIterations)

    def DrawStringCR(self, string):
        self.text_lines.append(string)
    
def main(test_class):
    """
    Loads the test class and executes it.
    """
    print "Loading %s..." % test_class.name
    test = test_class()
    if fwSettings.onlyInit:
        return
    test.run()

if __name__=="__main__":
    from test_empty import Empty
    main(Empty)
