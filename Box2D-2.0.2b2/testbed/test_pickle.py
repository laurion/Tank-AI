#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
import cPickle as pickle

# For using pickling with pybox2d in your own applications, there are several 
# things you should be aware of:
#  1. Saving the whole world is necessary. Shapes mean nothing without
#     bodies, which mean nothing without a world to put them in. You
#     can save your definitions, but that's additional overhead.
#  2. Save the state of your application also. The framework takes care
#     of all class variables in _pickle_vars. So, if you have 
#      _pickle_vars = ['a', 'b', 'c']
#     those variables will be pickled with the rest of the world. 
#     Bodies, shapes, joints, controllers all should work. If you need
#     more info on how to save manually, you can see Box2D.py itself.
#  3. ...

class Pickle(Framework):
    name = "Pickle" # Name of the class to display
    bodies = None # loaded from the example
    joints = None # loaded from the example
    _pickle_vars = ['bodies', 'joints']
    def __init__(self):
        super(Pickle, self).__init__()

        # load the example
        self.pickle_load('pickle_example_web')

        # for more info, see the pickle_load and pickle_save functions.

    def Step(self, settings):
        super(Pickle, self).Step(settings)
        self.DrawStringCR("So, does Pickling work?")

    # A couple examples of selective pickling.
    # These are unnecessary in normal save/load circumstances, but 
    # perhaps if you are writing an editor or something along those lines,
    # they will be of use.
    def pickle_save_no_joints(self, fn):
        """
        Save only bodies and controllers.

        Uses a bit of a hack to make the pickler think that there are 
        no joints.
        """

        self._pickle_vars=['bodies']

        backup_jointList = box2d.b2World.jointList
        box2d.b2World.jointList = []
        self.pickle_save(fn)
        box2d.b2World.jointList = backup_jointList

    def pickle_save_statics(self, fn):
        """
        Save only static bodies.

        Joints cannot be saved unless extra processing is done
        to ensure that all necessary bodies are saved with the
        joints.

        The ground body is saved regardless.
        """

        self._pickle_vars=[]

        # the pickler will skip the first body from the bodyList
        # because it assumes it to be the ground body. Always add
        # this None to your list first.
        my_bodylist=[None] 
        for body in self.world.bodyList[1:]: #skip the ground body
            if body.IsStatic():
                my_bodylist.append(body)

        backup_bodyList = box2d.b2World.bodyList
        backup_jointList = box2d.b2World.jointList

        box2d.b2World.jointList = []
        box2d.b2World.bodyList  = my_bodylist 

        self.pickle_save(fn)

        box2d.b2World.jointList = backup_jointList
        box2d.b2World.bodyList = backup_bodyList

    def Keyboard(self, key):
        # F5/F7 taken care of by the main testbed.
        # F5 saves to 'pickle_output', F7 loads the same file
        if key==K_F2:
            fn='pickle_output_Pickle_no_joints'
            self.pickle_save_no_joints(fn)
            self.pickle_load(fn)
            print 'Loaded joint count:', len(self.world.jointList)
        elif key==K_F3:
            fn='pickle_output_Pickle_only_statics'
            self.pickle_save_statics(fn)
            self.pickle_load(fn)
            print 'Loaded body count', len(self.world.bodyList)

if __name__=="__main__":
    main(Pickle)


