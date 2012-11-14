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

from settings import fwSettings

# Note that if you are making your own program and want it to be pyglet/pygame
# only, you can take 'pyglet_main.py' or 'pygame_main.py' and overwrite 
# 'test_main.py'.

known_backends = ("pyglet", "pygame", "cairo")

if fwSettings.backend in known_backends:
    exec("from %s_main import *" % fwSettings.backend)
else:
    from sys import argv

    print '''
Please set a proper "backend".
Either use the command line:
> %s --backend pygame
or edit the default settings in settings.py.
    ''' % (argv[0])
    exit(0)
