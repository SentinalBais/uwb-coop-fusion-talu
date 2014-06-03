#!/usr/bin/env python

import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_threshold_updater/src')
# importing plane fitting functions from traversability analysis functions

from uwb_fov import UWBFov
import matplotlib.pyplot as plt


def test1():
    ''' display the UWBFOV as a scatter plotter 
        ... does it look right 
    '''
    print "Test1"
    a = UWBFov(0.2,5,15)
    indexs = a.mask_indexs
    xpts = [point[0] for point in indexs]
    ypts = [point[1] for point in indexs]
    plt.plot(xpts, ypts, 'ro')
    plt.show()
    print 'Pass'

def main():
    print "Unit Testing"
    test1()

if __name__ == '__main__':
    main()