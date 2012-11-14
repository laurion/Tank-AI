# for releasing mpkg versions of pybox2d. not required for normal installations!

cd ..

# python 2.5
rm -rf dist build temp Box2D.py _Box2D.so
python2.5 setup.py build
/Library/Frameworks/Python.framework/Versions/2.5/bin/bdist_mpkg

# python 2.6
rm -rf dist build temp Box2D.py _Box2D.so
python2.6 setup.py build
/Library/Frameworks/Python.framework/Versions/2.6/bin/bdist_mpkg

cd misc
