#!/usr/bin/env python

import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_threshold_updater/src')
# importing plane fitting functions from traversability analysis functions
from uwb_thresh_updater import UWBThreshUpdater

def test1():
    ''' testing uwb_threshold_updater static functions 
    '''
    flag =  UWBThreshUpdater._missing_args(None,None,None)
    if flag == True:
        print 'pass'
    else:
        print 'fail'

def test2():
    ''' testing uwb_threshold_updater static functions 
    '''
    flag =  UWBThreshUpdater._missing_args([1,2,3],None,None)
    if flag == True:
        print 'pass'
    else:
        print 'fail'
 
def main():
    test1()
    test2()

if __name__ == '__main__':
    main()