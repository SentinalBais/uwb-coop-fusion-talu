#!/usr/bin/env python

import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
sys.path.append('/home/spartan3123/catkin_ws/src/utils_thesis/src/utils_thesis')
from scaling_functions import inverse_parabola

def test1():
    print 'Test1'
    s = inverse_parabola(20,40)
    if s == 0.5:
        print 'Pass'
    else:
        print 'Fail'

def test2():
    print 'Test2'
    s = inverse_parabola(0,40)
    if s == 1:
        print 'Pass'
    else:
        print 'Fail'

def main():
    print "Unit Testing"
    test1()
    test2()


if __name__ == '__main__':
    main()