#!/usr/bin/env python
import sys
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_threshold_updater/src/uwb_threshold_updater')

'''
    Testing how static methods can be called.... not really a unit test
'''

class A(object):
    def __init__(self):
        self.a = 'a'
        self.m1('b')

    @staticmethod
    def m1(a):
        print a

def test1():
    ''' testing uwb_threshold_updater static functions 
    '''
    a = A()
    a.m1('c')
    flag = True
    if flag == True:
        print 'pass'
    else:
        print 'fail'

def main():
    test1()

if __name__ == '__main__':
    main()