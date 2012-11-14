#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Setup script for pyBox2D distribution.

For installation instructions, see INSTALL.

Basic install steps:
 setup.py build

If that worked, then:
 setup.py install
"""

import os
from glob import glob

try:
    from setuptools import (setup, Extension)
    print 'Using setuptools.'
except:
    from distutils.core import (setup, Extension)
    print 'Setuptools not found; falling back on distutils.'

# release version number
box2d_version  = '2.0.2'
release_number = 2

# create the version string
version_str = "%sb%s" % (box2d_version, str(release_number))

def write_init(): 
    # read in the license header
    license_header = open(os.path.join('Box2D', 'pybox2d_license_header.txt')).read()

    # create the source code for the file
    init_source = [ "",
        "from Box2D import *",
        "__version__      = '%s'"    % version_str,
        "__version_info__ = (%s,%d)" % (box2d_version.replace('.', ','), release_number),
        ]

    # and create the __init__ file with the appropriate version string
    f=open('__init__.py', 'w')
    f.write(license_header)
    f.write( '\n'.join(init_source) )
    f.close()
    
source_paths = [
    'Box2D/Dynamics/',
    'Box2D/Dynamics/Contacts/',
    'Box2D/Dynamics/Controllers/',
    'Box2D/Dynamics/Joints/',
    'Box2D/Common/',
    'Box2D/Collision/',
    'Box2D/Collision/Shapes/',
    ]

# glob all of the paths and then flatten the list into one
box2d_source_files = ['Box2D/Box2D.i'] + \
    sum( [glob(os.path.join(path, "*.cpp")) for path in source_paths], [])

# arguments to pass to SWIG. for old versions of SWIG, -O (optimize) might not be present.
swig_arguments = '-c++ -IBox2D -O -small -includeall -ignoremissing -w201 -outdir .'

pybox2d_extension = \
    Extension('Box2D._Box2D', box2d_source_files, extra_compile_args=['-I.'], language='c++')

LONG_DESCRIPTION = \
""" 2D physics library Box2D %s for usage in Python.

    After installing please be sure to try out the testbed demos.
    They require either pygame or pyglet and are available on the
    homepage.

    pybox2d homepage: http://pybox2d.googlecode.com
    Box2D homepage: http://www.box2d.org
    """ % (box2d_version,)

CLASSIFIERS = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: zlib/libpng License",
    "Operating System :: Microsoft :: Windows",
    "Operating System :: MacOS :: MacOS X",
    "Operating System :: POSIX",
    "Programming Language :: Python",
    "Games :: Physics Libraries"
    ]

write_init()

setup_dict = dict(
    name             = "Box2D",
    version          = version_str,
    author           = "Ken Lauer",
    author_email     = "sirkne at gmail dot com",
    description      = "Python Box2D",
    license          = "zlib",
    url              ="http://pybox2d.googlecode.com/",
    long_description = LONG_DESCRIPTION,
    classifiers      = CLASSIFIERS,
    packages         = ['Box2D'],
    package_dir      = {'Box2D': '.'},
    options          = { 'build_ext': { 'swig_opts' : swig_arguments } },
    ext_modules      = [ pybox2d_extension ]
    )

# run the actual setup from distutils
setup( **setup_dict )
