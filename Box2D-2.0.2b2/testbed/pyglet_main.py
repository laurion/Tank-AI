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

"""
Keys:
    Space  - shoot projectile
    Z/X    - zoom
    Escape - quit

Other keys can be set by the individual test

Mouse:
    Left click  - select/drag body (creates mouse joint)
    Right click - pan
    Shift+Left  - drag to create a directional projectile
    Scroll      - zoom

You can easily add your own tests based on test_empty.

-kne
"""

import pyglet
from pyglet import gl
import Box2D as box2d
from settings import fwSettings
from pygame_keycodes import *
import math

class fwDestructionListener(box2d.b2DestructionListener):
    """
    The destruction listener callback:
    "SayGoodbye" is called when a joint or shape is deleted.
    """
    test = None
    def __init__(self):
        super(fwDestructionListener, self).__init__()

    def SayGoodbye(self, object):
        if isinstance(object, box2d.b2Joint):
            if self.test.mouseJoint==object:
                self.test.mouseJoint=None
            else:
                self.test.JointDestroyed(object)
        elif isinstance(object, box2d.b2Shape):
            self.test.ShapeDestroyed(object)

class fwBoundaryListener(box2d.b2BoundaryListener):
    """
    The boundary listener callback:
    Violation is called when the specified body leaves the world AABB.
    """
    test = None
    def __init__(self):
        super(fwBoundaryListener, self).__init__()

    def Violation(self, body):
        # So long as it's not the user-created bomb, let the test know that
        # the specific body has left the world AABB
        if self.test.bomb != body:
            self.test.BoundaryViolated(body)

class fwContactTypes:
    """
    Acts as an enum, holding the types necessary for contacts:
    Added, persisted, and removed
    """
    contactUnknown = 0
    contactAdded = 1
    contactPersisted = 2
    contactRemoved = 3

class fwContactPoint:
    """
    Structure holding the necessary information for a contact point.
    All of the information is copied from the contact listener callbacks.
    """
    shape1 = None
    shape2 = None
    normal = None
    position = None
    velocity = None
    id  = box2d.b2ContactID()
    state = 0

class fwContactListener(box2d.b2ContactListener):
    """
    Handles all of the contact states passed in from Box2D.

    """
    test = None
    def __init__(self):
        super(fwContactListener, self).__init__()

    def handleCall(self, state, point):
        if not self.test: return

        cp          = fwContactPoint()
        cp.shape1   = point.shape1
        cp.shape2   = point.shape2
        cp.position = point.position.copy()
        cp.normal   = point.normal.copy()
        cp.id       = point.id
        cp.state    = state
        self.test.points.append(cp)

    def Add(self, point):
        self.handleCall(fwContactTypes.contactAdded, point)

    def Persist(self, point):
        self.handleCall(fwContactTypes.contactPersisted, point)

    def Remove(self, point):
        self.handleCall(fwContactTypes.contactRemoved, point)

class grBlended (pyglet.graphics.Group):
    """
    This pyglet rendering group enables blending.
    """
    def set_state(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    def unset_state(self):
        gl.glDisable(gl.GL_BLEND)

class grPointSize (pyglet.graphics.Group):
    """
    This pyglet rendering group sets a specific point size.
    """
    def __init__(self, size=4.0):
        super(grPointSize, self).__init__()
        self.size = size
    def set_state(self):
        gl.glPointSize(self.size)
    def unset_state(self):
        gl.glPointSize(1.0)

class grText(pyglet.graphics.Group):
    """
    This pyglet rendering group sets the proper projection for
    displaying text when used.
    """
    window = None
    def __init__(self, window=None):
        super(grText, self).__init__()
        self.window = window

    def set_state(self):
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.gluOrtho2D(0, self.window.width, 0, self.window.height)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPushMatrix()
        gl.glLoadIdentity()

    def unset_state(self):
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)

class fwDebugDraw(box2d.b2DebugDraw):
    """
    This debug draw class accepts callbacks from Box2D (which specifies what to draw)
    and handles all of the rendering.

    If you are writing your own game, you likely will not want to use debug drawing.
    Debug drawing, as its name implies, is for debugging.
    """
    blended = grBlended()
    circle_segments = 16
    surface = None
    circle_cache_tf = {} # triangle fan (inside)
    circle_cache_ll = {} # line loop (border)
    def __init__(self): super(fwDebugDraw, self).__init__()
    def triangle_fan(self, vertices):
        """
        in: vertices arranged for gl_triangle_fan ((x,y),(x,y)...)
        out: vertices arranged for gl_triangles (x,y,x,y,x,y...)
        """
        out = []
        for i in range(1, len(vertices)-1):
            # 0,1,2   0,2,3  0,3,4 ..
            out.extend( vertices[0  ] )
            out.extend( vertices[i  ] )
            out.extend( vertices[i+1] )
        return len(out) / 2, out

    def line_loop(self, vertices):
        """
        in: vertices arranged for gl_line_loop ((x,y),(x,y)...)
        out: vertices arranged for gl_lines (x,y,x,y,x,y...)
        """
        out = []
        for i in range(0, len(vertices)-1):
            # 0,1  1,2  2,3 ... len-1,len  len,0
            out.extend( vertices[i  ] )
            out.extend( vertices[i+1] )
        
        out.extend( vertices[len(vertices)-1] )
        out.extend( vertices[0] )

        return len(out)/2, out

    def _getLLCircleVertices(self, radius, points):
        """
        Get the line loop-style vertices for a given circle.
        Drawn as lines.

        "Line Loop" is used as that's how the C++ code draws the
        vertices, with lines going around the circumference of the
        circle (GL_LINE_LOOP).

        This returns 'points' amount of lines approximating the 
        border of a circle.

        (x1, y1, x2, y2, x3, y3, ...)
        """
        ret = []
        step = 2*math.pi/points
        n = 0
        for i in range(0, points):
            ret.append( (math.cos(n) * radius, math.sin(n) * radius ) )
            n += step
            ret.append( (math.cos(n) * radius, math.sin(n) * radius ) )
        return ret

    def _getTFCircleVertices(self, radius, points):
        """
        Get the triangle fan-style vertices for a given circle.
        Drawn as triangles.

        "Triangle Fan" is used as that's how the C++ code draws the
        vertices, with triangles originating at the center of the
        circle, extending around to approximate a filled circle
        (GL_TRIANGLE_FAN).

        This returns 'points' amount of lines approximating the 
        circle.

        (a1, b1, c1, a2, b2, c2, ...)
        """
        ret = []
        step = 2*math.pi/points
        n = 0
        for i in range(0, points):
            ret.append( (0.0, 0.0) )
            ret.append( (math.cos(n) * radius, math.sin(n) * radius ) )
            n += step
            ret.append( (math.cos(n) * radius, math.sin(n) * radius ) )
        return ret

    def getCircleVertices(self, center, radius, points):
        """
        Returns the triangles that approximate the circle and
        the lines that border the circles edges, given
        (center, radius, points).

        Caches the calculated LL/TF vertices, but recalculates
        based on the center passed in.

        TODO: As of this point, there's only one point amount,
        so the circle cache ignores it when storing. Could cause 
        some confusion if you're using multiple point counts as
        only the first stored point-count for that radius will
        show up.

        Returns: (tf_vertices, ll_vertices)
        """
        if radius not in self.circle_cache_tf.keys():
            self.circle_cache_tf[radius]=self._getTFCircleVertices(radius,points)
            self.circle_cache_ll[radius]=self._getLLCircleVertices(radius,points)

        ret_tf, ret_ll = [], []

        for x, y in self.circle_cache_tf[radius]:
            ret_tf.extend( (x+center[0], y+center[1]) )
        for x, y in self.circle_cache_ll[radius]:
            ret_ll.extend( (x+center[0], y+center[1]) )
        return ret_tf, ret_ll

    def DrawCircle(self, center, radius, color):
        """
        Draw an unfilled circle given center, radius and color.
        """
        unused, ll_vertices = self.getCircleVertices( center, radius, self.circle_segments)
        ll_count = len(ll_vertices)/2

        self.batch.add(ll_count, gl.GL_LINES, None,
            ('v2f', ll_vertices),
            ('c4f', [color.r, color.g, color.b, 1.0] * (ll_count)))

    def DrawSolidCircle(self, center, radius, axis, color):
        """
        Draw an filled circle given center, radius, axis (of orientation) and color.
        """
        tf_vertices, ll_vertices = self.getCircleVertices( center, radius, self.circle_segments)
        tf_count, ll_count = len(tf_vertices) / 2, len(ll_vertices) / 2


        self.batch.add(tf_count, gl.GL_TRIANGLES, self.blended,
            ('v2f', tf_vertices),
            ('c4f', [0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5] * (tf_count)))

        self.batch.add(ll_count, gl.GL_LINES, None,
            ('v2f', ll_vertices),
            ('c4f', [color.r, color.g, color.b, 1.0] * (ll_count)))

        p = box2d.b2Vec2(axis)*radius+center
        self.batch.add(2, gl.GL_LINES, None,
            ('v2f', (center[0], center[1], p[0], p[1])),
            ('c3f', [1.0, 0.0, 0.0] * 2))

    def DrawPolygon(self, vertices, color):
        """
        Draw a wireframe polygon given the world vertices (tuples) with the specified color.
        """
        ll_count, ll_vertices = self.line_loop(vertices)

        self.batch.add(ll_count, gl.GL_LINES, None,
            ('v2f', ll_vertices),
            ('c4f', [color.r, color.g, color.b, 1.0] * (ll_count)))

    def DrawSolidPolygon(self, vertices, color):
        """
        Draw a wireframe polygon given the world vertices (tuples) with the specified color.
        """
        tf_count, tf_vertices = self.triangle_fan(vertices)

        self.batch.add(tf_count, gl.GL_TRIANGLES, self.blended,
            ('v2f', tf_vertices),
            ('c4f', [0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5] * (tf_count)))

        ll_count, ll_vertices = self.line_loop(vertices)

        self.batch.add(ll_count, gl.GL_LINES, None,
            ('v2f', ll_vertices),
            ('c4f', [color.r, color.g, color.b, 1.0] * (ll_count)))

    def DrawSegment(self, p1, p2, color):
        """
        Draw the line segment from p1-p2 with the specified color.
        """
        self.batch.add(2, gl.GL_LINES, None,
            ('v2f', (p1[0], p1[1], p2[0], p2[1])),
            ('c3f', [color.r, color.g, color.b]*2))

    def DrawXForm(self, xf):
        """
        Draw the transform xf on the screen
        """
        p1 = xf.position
        k_axisScale = 0.4
        p2 = p1 + k_axisScale * xf.R.col1
        p3 = p1 + k_axisScale * xf.R.col2

        self.batch.add(3, gl.GL_LINES, None,
            ('v2f', (p1[0], p1[1], p2[0], p2[1], p1[0], p1[1], p3[0], p3[1])),
            ('c3f', [1.0, 0.0, 0.0] * 2 + [0.0, 1.0, 0.0] * 2))

    def DrawPoint(self, p, size, color):
        """
        Draw a single point at point p given a point size and color.
        """
        self.batch.add(1, gl.GL_POINTS, grPointSize(size),
            ('v2f', (p[0], p[1])),
            ('c3f', [color.r, color.g, color.b]))
        
    def DrawAABB(self, aabb, color):
        """
        Draw a wireframe around the AABB with the given color.
        """
        self.debugDraw.batch.add(8, gl.GL_LINES, None,
            ('v2f', (aabb.lowerBound.x, aabb.lowerBound.y, abb.upperBound.x, aabb.lowerBound.y, 
                abb.upperBound.x, aabb.lowerBound.y, aabb.upperBound.x, aabb.upperBound.y,
                aabb.upperBound.x, aabb.upperBound.y, aabb.lowerBound.x, aabb.upperBound.y,
                aabb.lowerBound.x, aabb.upperBound.y, aabb.lowerBound.x, aabb.lowerBound.y)),
            ('c3f', [color.r, color.g, color.b] * 8))

class Framework(pyglet.window.Window):
    """
    The main testbed framework.
    It handles basically everything:
    * The initialization of pyglet, Box2D, and the window itself
    * Contains the main loop
    * Handles all user input.

    The window itself is derived from pyglet's Window, so you can use
    all of its functionality.

    You should derive your class from this one to implement your own tests.
    See test_Empty.py or any of the other tests for more information.
    """
    name = "None"

    # Box2D-related
    worldAABB          = None
    points             = []
    world              = None
    bomb               = None
    bombSpawning       = False
    bombSpawnPoint     = None
    points             = []
    mouseJoint         = None
    settings           = fwSettings
    mouseWorld         = None
    destroyList        = []

    # Box2D-callbacks
    destructionListener= None
    boundaryListener   = None
    contactListener    = None
    debugDraw          = None

    # Window-related
    fontname           = "Arial"
    fontsize           = 10
    font               = None
    textGroup          = None
    keys               = pyglet.window.key.KeyStateHandler()

    # Screen-related
    _viewZoom          = 1.0
    _viewCenter        = None
    screenSize         = None
    textLine           = 30
    font               = None
    fps                = 0

    def __init__(self, **kw):
        super(Framework, self).__init__(**kw)

        # Initialize the text display group
        self.textGroup = grText(self)

        # Load the font and record the screen dimensions
        self.font = pyglet.font.load(self.fontname, self.fontsize)
        self.screenSize = box2d.b2Vec2(self.width, self.height)

        # Box2D Initialization
        self.worldAABB = box2d.b2AABB()
        self.worldAABB.lowerBound = (-200.0, -100.0)
        self.worldAABB.upperBound = ( 200.0, 200.0)
        gravity = (0.0, -10.0)

        doSleep = True
        self.world = box2d.b2World(self.worldAABB, gravity, doSleep)
        self.destructionListener = fwDestructionListener()
        self.boundaryListener = fwBoundaryListener()
        self.contactListener = fwContactListener()
        self.debugDraw = fwDebugDraw()

        self.debugDraw.surface = self.screen

        self.destructionListener.test = self
        self.boundaryListener.test = self
        self.contactListener.test = self
        
        self.world.SetDestructionListener(self.destructionListener)
        self.world.SetBoundaryListener(self.boundaryListener)
        self.world.SetContactListener(self.contactListener)
        self.world.SetDebugDraw(self.debugDraw)

        self._viewCenter = box2d.b2Vec2(0,10.0)

    def on_close(self):
        """
        Callback: user tried to close the window
        """
        pyglet.clock.unschedule(self.SimulationLoop)
        super(Framework, self).on_close()

    def on_show(self):
        """
        Callback: the window was shown.
        """
        self.updateProjection()

    def updateProjection(self):
        """
        Recalculates the necessary projection.
        """
        gl.glViewport(0, 0, self.width, self.height)

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()

        ratio = float(self.width) / self.height

        extents = box2d.b2Vec2(ratio * 25.0, 25.0)
        extents *= self._viewZoom

        lower = self._viewCenter - extents
        upper = self._viewCenter + extents

        # L/R/B/T
        gl.gluOrtho2D(lower.x, upper.x, lower.y, upper.y)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

    def setCenter(self, value):
        """
        Updates the view offset based on the center of the screen.
        """
        if isinstance(value, box2d.b2Vec2):
            self._viewCenter = value.copy()
        elif isinstance(value, (list, tuple)):
            self._viewCenter = box2d.b2Vec2( *value )
        else:
            raise ValueError, 'Expected b2Vec2 or sequence'

        self.updateProjection()

    def setZoom(self, zoom):
        self._viewZoom = zoom
        self.updateProjection()

    viewZoom   = property(lambda self: self._viewZoom, setZoom,
                           doc='Zoom factor for the display')
    viewCenter = property(lambda self: self._viewCenter, setCenter, 
                           doc='Screen center in camera coordinates')

    def on_key_press(self, key, modifiers):
        """
        Checks for the initial keydown of the basic testbed keys. Passes the unused
        ones onto the test via the Keyboard() function.
        """
        if key==pyglet.window.key.ESCAPE:
            exit(0)
        elif key==pyglet.window.key.Z:
            # Zoom in
            self.viewZoom = min(1.1 * self.viewZoom, 20.0)
        elif key==pyglet.window.key.X:
            # Zoom out
            self.viewZoom = max(0.9 * self.viewZoom, 0.02)
        elif key==pyglet.window.key.R:
            # Reload (disabled)
            #print "Reload not functional"
            exit(10)
        elif key==pyglet.window.key.SPACE:
            # Launch a bomb
            self.LaunchRandomBomb()
        elif key==pyglet.window.key.F5:    # Save state
            self.pickle_save('pickle_output')
        elif key==pyglet.window.key.F7:    # Load state
            self.pickle_load('pickle_output')
        else:
            # Inform the test of the key press
            self.Keyboard(key)

    def on_mouse_motion(self, x, y, dx, dy):
        self.invalid=False

    def on_mouse_press(self, x, y, button, modifiers):
        """
        Mouse down
        """
        p = self.ConvertScreenToWorld(x, y)
        self.mouseWorld = p
        if button == pyglet.window.mouse.LEFT:
            if modifiers & pyglet.window.key.MOD_SHIFT:
                self.ShiftMouseDown( p )
            else:
                self.MouseDown( p )
        elif button == pyglet.window.mouse.MIDDLE:
            pass

    def on_mouse_release(self, x, y, button, modifiers):
        """
        Mouse up
        """
        p = self.ConvertScreenToWorld(x, y)
        self.mouseWorld = p

        if button == pyglet.window.mouse.LEFT:
            self.MouseUp(p)

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        """
        Mouse scrollwheel used
        """
        if scroll_y < 0:
            self.viewZoom *= 1.1
        elif scroll_y > 0:
            self.viewZoom /= 1.1

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        """
        Mouse moved while clicking
        """
        p = self.ConvertScreenToWorld(x, y)
        self.mouseWorld = p

        self.MouseMove(p)

        if buttons & pyglet.window.mouse.RIGHT:
            self.viewCenter -= (float(dx)/5, float(dy)/5)

    def pickle_load(self, fn, set_vars=True, additional_vars=[]):
        """
        Load the pickled world in file fn.
        additional_vars is a dictionary to be populated with the
        loaded variables.
        """
        import cPickle as pickle
        try:
            world, variables = pickle.load(open(fn, 'rb'))
        except Exception, s:
            print 'Error while loading world: ', s
            return
        
        self.world = world._pickle_finalize()
        variables=box2d.pickle_fix(self.world, variables, 'load')

        if set_vars:
            for var, value in variables.items():
                if hasattr(self, var):
                    setattr(self, var, value)
                else:
                    print 'Unknown property %s=%s' % (var, value)

        self.bomb = None
        self.bombSpawning = False

        # have to reset a few things that can't be saved:
        self.world.SetDestructionListener(self.destructionListener)
        self.world.SetBoundaryListener(self.boundaryListener)
        self.world.SetContactListener(self.contactListener)
        self.world.SetDebugDraw(self.debugDraw)
        print 'Loaded'

        return variables


    def pickle_save(self, fn, additional_vars={}):
        import cPickle as pickle
        if self.mouseJoint:
            self.MouseUp(self.mouseWorld) # remove a mouse joint if it exists
    
        if not additional_vars and hasattr(self, '_pickle_vars'):
            additional_vars=dict((var, getattr(self, var)) for var in self._pickle_vars)

        save_values = [self.world, box2d.pickle_fix(self.world, additional_vars, 'save')]

        try:
            pickle.dump(save_values, open(fn, 'wb'))
        except Exception, s:
            print 'Pickling failed: ', s
            return

        print 'Saved'

    def run(self):
        """
        Main loop.
        """
        if self.settings.hz > 0.0:
            pyglet.clock.schedule_interval(self.SimulationLoop, 1.0 / self.settings.hz)
        pyglet.app.run()

    def SetTextLine(self, line):
        """
        Kept for compatibility with C++ Box2D's testbeds.
        """
        self.textLine=line

    def Step(self, settings):
        """
        The main physics step.

        Takes care of physics drawing (callbacks are executed after the world.Step() )
        and drawing additional information.
        """

        # Don't do anything if the setting's Hz are <= 0
        if settings.hz > 0.0:
            timeStep = 1.0 / settings.hz
        else:
            timeStep = 0.0
        
        # If paused, display so
        if settings.pause:
            if settings.singleStep:
                settings.singleStep=False
            else:
                timeStep = 0.0

            self.DrawStringCR("****PAUSED****")

        # Set the flags based on what the settings show (uses a bitwise or mask)
        flags = 0
        if settings.drawShapes:     flags |= box2d.b2DebugDraw.e_shapeBit
        if settings.drawJoints:     flags |= box2d.b2DebugDraw.e_jointBit
        if settings.drawControllers:flags |= box2d.b2DebugDraw.e_controllerBit
        if settings.drawCoreShapes: flags |= box2d.b2DebugDraw.e_coreShapeBit
        if settings.drawAABBs:      flags |= box2d.b2DebugDraw.e_aabbBit
        if settings.drawOBBs:       flags |= box2d.b2DebugDraw.e_obbBit
        if settings.drawPairs:      flags |= box2d.b2DebugDraw.e_pairBit
        if settings.drawCOMs:       flags |= box2d.b2DebugDraw.e_centerOfMassBit
        self.debugDraw.SetFlags(flags)

        # Set the other settings that aren't contained in the flags
        self.world.SetWarmStarting(settings.enableWarmStarting)
    	self.world.SetContinuousPhysics(settings.enableTOI)

        # Reset the collision points
        self.points = []

        # Tell Box2D to step
        self.world.Step(timeStep, settings.velocityIterations, settings.positionIterations)
        self.world.Validate()

        # Destroy bodies that have left the world AABB (can be removed if not using pickling)
        for obj in self.destroyList:
            self.world.DestroyBody(obj)
        self.destroyList = []

        # If the bomb is frozen, get rid of it.
        if self.bomb and self.bomb.IsFrozen():
            self.world.DestroyBody(self.bomb)
            self.bomb = None

        if settings.drawStats:
            self.DrawStringCR("proxies(max) = %d(%d), pairs(max) = %d(%d)" % (
                self.world.GetProxyCount(), box2d.b2_maxProxies, self.world.GetPairCount(), box2d.b2_maxPairs) )

            self.DrawStringCR("bodies/contacts/joints = %d/%d/%d" %
                (self.world.GetBodyCount(), self.world.GetContactCount(), self.world.GetJointCount()))

            self.DrawStringCR("hz %d vel/pos iterations %d/%d" %
                (settings.hz, settings.velocityIterations, settings.positionIterations))

            self.DrawStringCR("heap bytes = %d" % box2d.cvar.b2_byteCount)

        if settings.drawFPS: #python version only
            self.DrawStringCR("FPS %d" % self.fps)
        
        # If there's a mouse joint, draw the connection between the object and the current pointer position.
        if self.mouseJoint:
            body = self.mouseJoint.GetBody2()
            p1 = body.GetWorldPoint(self.mouseJoint.localAnchor)
            p2 = self.mouseJoint.target

            self.debugDraw.DrawPoint(p1, settings.pointSize, box2d.b2Color(0,1.0,0))
            self.debugDraw.DrawPoint(p2, settings.pointSize, box2d.b2Color(0,1.0,0))
            self.debugDraw.DrawSegment(p1, p2, box2d.b2Color(0.8,0.8,0.8))

        # Draw the slingshot bomb
        if self.bombSpawning:
            self.debugDraw.DrawPoint(self.bombSpawnPoint, settings.pointSize, box2d.b2Color(0,0,1.0))
            self.debugDraw.DrawSegment(self.bombSpawnPoint, self.mouseWorld, box2d.b2Color(0.8,0.8,0.8))

        # Draw each of the contact points in different colors.
        if self.settings.drawContactPoints:
            #k_impulseScale = 0.1
            k_axisScale = 0.3

            for point in self.points:
                if point.state == fwContactTypes.contactAdded:
                    self.debugDraw.DrawPoint(point.position, settings.pointSize, box2d.b2Color(0.3, 0.95, 0.3))
                elif point.state == fwContactTypes.contactPersisted:
                    self.debugDraw.DrawPoint(point.position, settings.pointSize, box2d.b2Color(0.3, 0.3, 0.95))
                else: #elif point.state == fwContactTypes.contactRemoved:
                    self.debugDraw.DrawPoint(point.position, settings.pointSize, box2d.b2Color(0.95, 0.3, 0.3))

                if settings.drawContactNormals:
                    p1 = point.position
                    p2 = p1 + k_axisScale * point.normal
                    self.debugDraw.DrawSegment(p1, p2, box2d.b2Color(0.4, 0.9, 0.4))

    def LaunchBomb(self, position, velocity):
        """
        A bomb is a simple circle which has the specified position and velocity.
        """
        if self.bomb:
            self.world.DestroyBody(self.bomb)
            self.bomb = None

        bd = box2d.b2BodyDef()
        bd.allowSleep = True
        bd.position = position
        bd.isBullet = True
        self.bomb = self.world.CreateBody(bd)
        self.bomb.SetLinearVelocity(velocity)

        sd = box2d.b2CircleDef()
        sd.radius = 0.3
        sd.density = 20.0
        sd.restitution = 0.1

        minV = position - box2d.b2Vec2(0.3,0.3)
        maxV = position + box2d.b2Vec2(0.3,0.3)

        aabb = box2d.b2AABB()
        aabb.lowerBound = minV
        aabb.upperBound = maxV

        if self.world.InRange(aabb):
            self.bomb.CreateShape(sd)
            self.bomb.SetMassFromShapes()

    def LaunchRandomBomb(self):
        """
        Create a new bomb and launch it at the testbed.
        """
        p = box2d.b2Vec2( box2d.b2Random(-15.0, 15.0), 30.0 )
        v = -5.0 * p
        self.LaunchBomb(p, v)
     
    def CheckKeys(self):
        """
        Check the keys that are evaluated on every main loop iteration.
        I.e., they aren't just evaluated when first pressed down
        """
        if self.keys[pyglet.window.key.LEFT]:
            self.viewCenter.x -= 0.5
        elif self.keys[pyglet.window.key.RIGHT]:
            self.viewCenter.x += 0.5
        if self.keys[pyglet.window.key.UP]:
            self.viewCenter.y += 0.5
        elif self.keys[pyglet.window.key.DOWN]:
            self.viewCenter.y -= 0.5

        if self.keys[pyglet.window.key.HOME]:
            self._viewZoom = 1.0 
            self.viewCenter = (0.0, 20.0)

    def SimulationLoop(self, dt):
        """
        The main simulation loop. Don't override this, override Step instead.
        And be sure to call super(classname, self).Step(settings) at the end
        of your Step function.
        """

        # Check the input
        self.CheckKeys()

        # Clear the screen
        self.clear()

        # Update the keyboard status
        self.push_handlers(self.keys)

        # Create a new batch for drawing
        self.debugDraw.batch = pyglet.graphics.Batch()

        # Reset the text position
        self.SetTextLine(15)

        # Draw the title of the test at the top
        self.DrawStringCR(self.name)

        # Step the physics
        self.Step(self.settings)

        self.debugDraw.batch.draw()
        self.invalid = True

        self.fps = pyglet.clock.get_fps()

    def ConvertScreenToWorld(self, x, y):
        """
        Takes screen (x, y) and returns
        world coordinate b2Vec2(x,y).
        """
        u = float(x) / self.width
        v = float(y) / self.height

        ratio = float(self.width) / self.height
        extents = box2d.b2Vec2(ratio * 25.0, 25.0)
        extents *= self._viewZoom

        lower = self._viewCenter - extents
        upper = self._viewCenter + extents

        p = box2d.b2Vec2( 
            (1.0 - u) * lower.x + u * upper.x,
            (1.0 - v) * lower.y + v * upper.y )

        return p


    def DrawString(self, x, y, str, color=(229,153,153,255)):
        """
        Draw some text, str, at screen coordinates (x, y).
        """
        text = pyglet.text.Label(str, font_name=self.fontname, font_size=self.fontsize, 
                                 x=x, y=self.height-y, color=color, batch=self.debugDraw.batch, group=self.textGroup)

    def DrawStringCR(self, str, color=(229,153,153,255)):
        """
        Draw some text, str, at screen coordinates (x, y).
        """
        text = pyglet.text.Label(str, font_name=self.fontname, font_size=self.fontsize, 
                      x=5, y=self.height-self.textLine, color=color, batch=self.debugDraw.batch, group=self.textGroup)
        self.textLine += 15

    def ShiftMouseDown(self, p):
        """
        Indicates that there was a left click at point p (world coordinates) with the
        left shift key being held down.
        """
        self.mouseWorld = p

        if self.mouseJoint != None:
            return

        self.SpawnBomb(p)

    def MouseDown(self, p):
        """
        Indicates that there was a left click at point p (world coordinates)
        """
        if self.mouseJoint != None:
            return

        # Make a small box.
        aabb = box2d.b2AABB()
        d = box2d.b2Vec2(0.001, 0.001)
        aabb.lowerBound = p - d
        aabb.upperBound = p + d

        # Query the world for overlapping shapes.
        body = None
        k_maxCount = 10

        (count, shapes) = self.world.Query(aabb, k_maxCount)
        for shape in shapes:
            shapeBody = shape.GetBody()
            if shapeBody.IsStatic() == False and shapeBody.GetMass() > 0.0:
                if shape.TestPoint(shapeBody.GetXForm(), p): # is it inside?
                    body = shapeBody
                    break
        
        if body:
            md = box2d.b2MouseJointDef()
            md.body1   = self.world.GetGroundBody()
            md.body2   = body
            md.target  = p
            md.maxForce= 1000.0 * body.GetMass()
            self.mouseJoint = self.world.CreateJoint(md)
            body.WakeUp()
            
    def MouseUp(self, p):
        """
        Left mouse button up.
        """     
        if self.mouseJoint:
            self.world.DestroyJoint(self.mouseJoint)
            self.mouseJoint = None

        if self.bombSpawning:
            self.CompleteBombSpawn(p)

    def MouseMove(self, p):
        """
        Mouse moved to point p, in world coordinates.
        """
        if self.mouseJoint:
            self.mouseJoint.SetTarget(p)

    def SpawnBomb(self, worldPt):
        """
        Begins the slingshot bomb by recording the initial position.
        Once the user drags the mouse and releases it, then 
        CompleteBombSpawn will be called and the actual bomb will be
        released.
        """
        self.bombSpawnPoint = worldPt.copy()
        self.bombSpawning = True

    def CompleteBombSpawn(self, p):
        """
        Create the slingshot bomb based on the two points
        (from the worldPt passed to SpawnBomb to p passed in here)
        """
        if not self.bombSpawning: 
            return
        multiplier = 30.0
        vel  = self.bombSpawnPoint - p
        vel *= multiplier
        self.LaunchBomb(self.bombSpawnPoint, vel)
        self.bombSpawning = False

    # These should be implemented in the subclass: (Step() also if necessary)
    def ShapeDestroyed(self, joint):
        pass

    def JointDestroyed(self, joint):
        pass

    def BoundaryViolated(self, body):
        # Be sure to check if any of these bodies are ones your game
        # stores. Using a reference to a deleted object will cause a crash.
        # e.g., 
        # if body==self.player: 
        #     self.player=None
        #
        # The following checks for generic usage by testing the pickle variables.

        if hasattr(self, '_pickle_vars'):
            for var in self._pickle_vars:
                value=getattr(self, var)
                if body==value:
                    setattr(self, var, None)
                elif isinstance(value, list):
                    if body in value:
                        value.remove(body)

        # Not destroying bodies outside the world AABB will cause
        # pickling to fail, so destroy it after the next step:
        self.destroyList.append(body)

    def Keyboard(self, key):
        pass

def main(test_class):
    print "Loading %s..." % test_class.name
    test = test_class()
    if fwSettings.onlyInit:
        return
    test.run()

if __name__=="__main__":
    from test_empty import Empty
    main(Empty)
