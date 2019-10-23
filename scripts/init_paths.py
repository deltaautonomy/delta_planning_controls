#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Oct 23, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Built-in modules
import os
import sys
import inspect
import platform
import os.path as osp


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

# Display Python version
python_version = platform.python_version()
print('Python', python_version)

# Handle OpenCV
ROS_CV = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if python_version.startswith('3'):
    if ROS_CV in sys.path: sys.path.remove(ROS_CV)
    import cv2
    print('OpenCV Version (Build Py3):', cv2.__version__)
    add_path(ROS_CV)
else:
    import cv2
    print('OpenCV Version (ROS):', cv2.__version__)

# Import useful packages
import math
import time
import pickle
import random
import shutil
import numpy as np

# Handle paths
THIS_PATH = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
PKG_PATH = os.path.dirname(THIS_PATH)
add_path(PKG_PATH)

# Display paths
print('Package Path:', PKG_PATH)

# Setup complete
print('\n\033[95m' + '*' * 30 + ' Delta Alerts ' + '*' * 30 + '\033[00m\n')
