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

class fwSettings(object):
    backend='pygame'        # The default backend to use in ('pyglet', 'pygame')

    # Physics options
    hz=60.0
    velocityIterations=10
    positionIterations=8
    enableWarmStarting=True   # Makes physics results more accurate (see Box2D wiki)
    enableTOI=True            # Calculate time of impact

    # Drawing
    drawStats=False
    drawShapes=True
    drawControllers=True
    drawJoints=True
    drawCoreShapes=False
    drawAABBs=False
    drawOBBs=False
    drawPairs=False
    drawContactPoints=False
    drawContactNormals=False
    drawContactForces=False
    drawFrictionForces=False
    drawFPS=True
    drawMenu=True             # toggle by pressing F1
    drawCOMs=False            # Centers of mass
    pointSize=2.5             # pixel radius for drawing points

    # Miscellaneous testbed options
    pause=False
    singleStep=False
    onlyInit=False            # run the test's initialization without graphics, and then quit (for testing)

from optparse import OptionParser

parser = OptionParser()
list_options = [i for i in dir(fwSettings) if not i.startswith('_')]

for opt_name in list_options:
    value = getattr(fwSettings, opt_name)
    if isinstance(value, bool):
        if value:
            parser.add_option('','--NO'+opt_name, dest=opt_name, default=value,
                              action='store_'+str(not value).lower(),
                              help="don't "+opt_name)
        else:
            parser.add_option('','--'+opt_name, dest=opt_name, default=value,
                              action='store_'+str(not value).lower(),
                              help=opt_name)
            
    else:
        if isinstance(value, int):
            opttype = 'int'
        elif isinstance(value, float):
            opttype = 'float'
        else:
            opttype = 'string'
        parser.add_option('','--'+opt_name, dest=opt_name, default=value,
                          type=opttype,
                          help='sets the %s option'%(opt_name,))


(fwSettings, args) = parser.parse_args()


