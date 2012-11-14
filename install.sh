#!/bin/bash
clear
chmod +x main.py
chmod +x Tank-AI.exe
echo "Installing Tank-AI dependencies"
sudo easy_install pygame pygsear
sudo apt-get install build-essential python-dev swig
cd Box2D-2.0.2b2
python setup.py build
sudo python setup.py install
cd ..


