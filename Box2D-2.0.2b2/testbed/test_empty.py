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

from test_main import *

class Empty(Framework):
    """You can use this class as an outline for your tests.

    """
    name = "Empty" # Name of the class to display
    _pickle_vars=[] # variables to pickle (save/load). e.g., ['name', 'var1', 'var2']
    def __init__(self):
        """ 
        Initialize all of your objects here.
        Be sure to call the Framework's initializer first.
        """
        super(Empty, self).__init__()

        # Initialize all of the objects


    def Keyboard(self, key):
        """
        The key is from pygame.locals.K_*
        (e.g., if key == K_z: ... )

        If you are using the pyglet backend, you should be able to use the same
        K_[a-z], see pyglet_keymapper.py
        """
        pass

    def Step(self, settings):
        """Called upon every step.
        You should always call
         -> super(Your_Test_Class, self).Step(settings)
        at the beginning or end of your function.

        If placed at the beginning, it will cause the actual physics step to happen first.
        If placed at the end, it will cause the physics step to happen after your code.
        """

        super(Empty, self).Step(settings)

        # do stuff

        # Placed after the physics step, it will draw on top of physics objects
        self.DrawStringCR("*** Base your own testbeds on me! ***")

    def ShapeDestroyed(self, shape):
        """
        Callback indicating 'shape' has been destroyed.
        """
        pass

    def JointDestroyed(self, joint):
        """
        The joint passed in was removed.
        """
        pass

    #def BoundaryViolated(self, body):
    #    """
    #    The body went out of the world's extents.
    #    """
    #    See pygame_main's implementation of BoundaryViolated for more information
    #    about pickling and general stability.

if __name__=="__main__":
    main(Empty)

