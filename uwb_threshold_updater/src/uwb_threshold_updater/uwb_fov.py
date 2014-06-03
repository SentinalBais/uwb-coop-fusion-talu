#!/usr/bin/env python 
import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
from data_structures.point import Point
# importing plane fitting functions from traversability analysis functions
sys.path.append('/home/spartan3123/catkin_ws/src/traversability_analysis/src/traversability_index/functions')


import numpy as np
import math as m

'''
    Class to represent the UWBRadar FOV 
'''
class UWBFov(object):
    def __init__(self, cell_size, rmin=5.1, rmax=16.5, range_res=0.3, beamwidth=40):

        self.rmin = rmin
        self.rmax = rmax
        self.range_res = range_res
        self.beamwidth = beamwidth
        self.mask_indexs = []
        self.index_spacing = cell_size
        self.__determine_mask()


    def bin_index(self, true_range):

        index = int(m.floor(true_range/self.range_res))
        return index

    def __determine_mask(self):
        '''
            Determines the 2D indexes which represents the FOV of the UWB radar. Assumes that the vertical beamwidth is such that
            the radar beam intersects the ground before Rmin, if not the vertical beamwidth needs to be considered. During testing
            in relatively smooth terrain of the ACFR lawns this will not happend

            Indexs will be relative to the sensor frame

            ^ (x axis)
            |
            |
            |
            ___________>(y axis) 

            This function should only be called once ( by the constructor) 
        '''

        start_y = -self.rmax/2.0
        end_y = self.rmax/2.0

        start_x = 0
        end_x = self.rmax

        azi_min = -self.beamwidth/2*m.pi/180.0
        azi_max = self.beamwidth/2*m.pi/180.0

        '''
            Iterate around a grid that surrounds the UWB FOV and determine if a point
            is in the UWB radar FOV  
        '''
        # add the sensor origin at 0,0,0
        sensor_point = Point(0,0,0)
        self.mask_indexs.append([sensor_point.x, sensor_point.y, sensor_point.range, sensor_point.azimuth, sensor_point.elvation])
        for x in np.arange(start_x, end_x, self.index_spacing):
            for y in np.arange(start_y, end_y, self.index_spacing):
                point = Point(x, y, 0)
                if point.range >= self.rmin and point.range <= self.rmax:
                    if point.azimuth >= azi_min and point.azimuth <= azi_max:
                        # then the point is in the UWB radar FOV 
                        self.mask_indexs.append([point.x, point.y, point.range, point.azimuth, point.elvation])








