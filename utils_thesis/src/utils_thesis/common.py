#!/usr/bin/env python
import rospy 
from point import Point

def get_xy_limits(fname):
    '''
        Reads a pcd_file and determines the min/max of x and y 
    ''' 
    data_start = False
    x_max = -99999.0
    x_min = 999999.0
    y_max = -99999.0
    y_min = 999999.0
    with open(fname, 'r') as f:
        for i, line in enumerate(f):
            words = line.split()
            if words[0] == "DATA":
                data_start = True
            elif data_start == True:
                x, y = (float(words[0]), float(words[1]) )
                if x > x_max:
                    x_max = x
                if x < x_min:
                    x_min = x
                if y > y_max:
                    y_max = y
                if y < y_min:
                    y_min = y
    return [x_max, x_min, y_max, y_min]

def read_pcd(fname, emap):
    '''
       Reads a pcd_file and enters the points into an elevation map 
    '''
    data_start = False
    with open(fname, 'r') as f:
        for i, line in enumerate(f):
            words = line.split()
            if words[0] == "DATA":
                data_start = True
            elif data_start == True:
                point = Point(float(words[0]), float(words[1]), float(words[2]))
                emap.addPoint(point)

def update_time_stamp(msg):
    msg.header.stamp = rospy.Time.now()

def update_time_stamp_zero(msg):
    '''
        Sets the time the time stamp to 0,0
    '''
    msg.header.stamp = rospy.Time(0.0)



