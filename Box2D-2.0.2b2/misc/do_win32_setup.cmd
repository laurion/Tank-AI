
@echo This script creates release installers for the supported versions of Python
@echo in Windows. Though it's a windows command script, it does uses some stuff
@echo from MSYS (cat/mv/rm/etc)

@pause

cd ..

@echo Clean all first...
c:\python24\python.exe setup.py clean -a
c:\python25\python.exe setup.py clean -a
c:\python26\python.exe setup.py clean -a
rm -rf build

@echo ---------------------------------------------------------------------------
@echo Python 2.4 - MinGW
c:\python24\python.exe setup.py build --force
cat Box2D/pybox2d_license_header.txt > Box2D_.py
cat Box2D.py >> Box2D_.py
mv Box2D_.py Box2D.py
c:\python24\python.exe setup.py bdist_wininst -t"pyBox2D Installer" -dinstaller

@echo ---------------------------------------------------------------------------
@echo Python 2.5 - MinGW
c:\python25\python.exe setup.py build --force
cat Box2D/pybox2d_license_header.txt > Box2D_.py
cat Box2D.py >> Box2D_.py
mv Box2D_.py Box2D.py
c:\python25\python.exe setup.py bdist_wininst -t"pyBox2D Installer" -dinstaller

@echo ---------------------------------------------------------------------------
@echo Python 2.6 - Compiles with VC9
c:\python26\python.exe setup.py build --force
cat Box2D/pybox2d_license_header.txt > Box2D_.py
cat Box2D.py >> Box2D_.py
mv Box2D_.py Box2D.py
c:\python26\python.exe setup.py bdist_wininst -t"pyBox2D Installer" -dinstaller
rm -rf build temp Box2D_wrap* Box2D*.py _Box2D.pyd

cd misc

@pause


