#!/usr/bin/env python
import sys
sys.path.append('/home/spartan3123/catkin_ws/src/traversability_analysis/src/data_structures')
import numpy as np
import math as m
import rospy
import tf
from functions.plane_fit import plane_fit
from functions.plane_fit import slope_plane
from functions.plane_fit import z_value_plane
from meta_cell import MetaCell
from nav_msgs.msg import OccupancyGrid


class TraversabilityIndex(object):
    def __init__(self, emap, cell_min_cap=10):
        self.emap = emap
        self.delta_x = emap.delta_x
        self.delta_y = emap.delta_y
        self.cell_size = emap.cell_size
        # the minium number of points per cell, for a cell to be considered known 
        self.cell_min_cap = cell_min_cap
        self.cell_size = emap.cell_size

        # Metrics statistics 
        self.max_slope = -1 
        self.max_height = -1
        self.max_roughness = -1 
        self.MAX_ROUGHNESS = 1 
        # percentage weighting for metrics
        self.M1P = 0.6
        self.M2P = 0.4
        ''' The untraversablity constant (UTC) was approximated using the formula 
            estimate max traversable slope for mantis: 30 degrees (m_slope)
            estimate max traversable roughness for mantis: 0.5  (m_r)

            UTC = (m_slope/max_slope*M1P + m_r/max_roughness*M2P)*100

            where max_slope and max_roughness the upper bound limits for the 
            metric function. If a metric function is unbounded the upper bound is set such that
            it would be impossible for any vehicle to traverse terrain with the 
            max metric value

            The UTC value can be tunned if required, it is to be adjusted untile
            the traversability map is representative of what the vehicle can traverse
        ''' 
        self.UTC = 28
        # constants if method [13] if follwed to the letter 
        self.F1 = 200
        self.F2 = 30

        # CONSTANTS
        EDGE_PADDING = 20
        # size of the mask applied to the elevation map
        # this variable should be odd  
        self.mask_length = 3

        # intilise the traversability map
        # x axis || collumns, each row is a new x value, x_index = row_index
        # y axis || rows, each col is a new y value  y_index = col_index
        ''' (traversability map coordinate frame)
        (x axis)
        ^
        |
        |
        *_________> (y axis)
        '''
        self.num_rows = int(self.delta_x/self.cell_size) + EDGE_PADDING
        self.num_cols = int(self.delta_y/self.cell_size) + EDGE_PADDING
        # Initlise traversability map 
        # 0-100 represent traversability, where 0 is traversable and 100 is untraversable
        # -1 represents unknown cells
        # intilise traversability map to be uknown 
        self.t_map = np.ones(shape=(self.num_rows ,self.num_cols), dtype=float)*-1
        # intilise the meta data map with empty cells 
        meta_map = [[MetaCell() for i in xrange(self.num_cols)] for j in xrange(self.num_rows)]
        self.m_map = np.array(meta_map)


    def eval_metrics(self):
        # apply a mask over the elevation map, fitting planes to a LxL mask 
        for row_index in xrange(self.emap.map.shape[0]):
            for col_index in xrange (self.emap.map.shape[1]):
                if self.emap.map[row_index, col_index].known == True:
                    # get x,y,z values using a LxL mask that is centred at row_i,col_i 
                    # where the x,y values are the cell centres in cartesian coordinates 
                    # and the z value is the z_min of each cell
                    [xl, yl, zl] = self._mask_cells(row_index, col_index)
                    if len(xl) == self.mask_length*self.mask_length:
                        # get the equation of the plane that fits the points xl, yl, zl
                        P = plane_fit(xl=xl, yl=yl, zl=zl)
                        # P = [A,B,C,D] coeficients of the equation of a plane
                        # Get the normal vector
                        N = P[0:3]
                        # metric 1: determine the slope of the fit plane relative to the x,y plane 
                        slope = slope_plane(N)
                        # convert the angle into the accute angle, between the x,y plane and the fit plane
                        if slope > m.pi/2:
                            slope = slope - m.pi
                        # determine the roughness metric 
                        roughness = self._roughness_metric(P, row_index, col_index)
                        # get the x,y location of the cell
                        [x, y] = self.emap.index_to_cart(row_index, col_index)
                        # get the heighest elevation bellonging to the cell 
                        z = self.emap.map[row_index, col_index].z_value_min
                        # determine hieght of centre cell above the fit ground plane 
                        height_above_local_plane = abs(z - z_value_plane(x, y, P))
                        # encode metric results into the meta data map 
                        self._set_metrics(row_index, col_index, slope, height_above_local_plane, roughness)
                    
    def _set_metrics(self, row_index, col_index, slope, height, roughness):
        self.m_map[row_index, col_index].set_metrics(slope, height, roughness)
        # update the global maxium values for the metrics, which may be used for normalisation 
        if slope > self.max_slope:
            self.max_slope = slope 
        if height > self.max_height:
            self.max_height = height
        if roughness > self.max_roughness: 
            self.max_roughness = roughness


    def eval_t_index(self):
        '''
            Evaluates the traversability index given the metric information in the meta data map  
        '''
        for row_index in xrange(self.m_map.shape[0]):
            for col_index in xrange(self.m_map.shape[1]):
                if self.m_map[row_index, col_index].known == True:
                    slope = self.m_map[row_index, col_index].slope
                    roughness = self.m_map[row_index, col_index].roughness_metric
                    # normalise metric values - using estimate max limit and saturate if required.
                    max_slope = m.pi/2
                    # no max_limit exists for the roughness metric, therefore one is created such that
                    # max_roughness value could be considered untrevserable for any vehicle 

                    # normalise slope
                    n_slope = abs(slope/max_slope)
                    # normalise and saturate roughness
                    roughness = abs(roughness/self.MAX_ROUGHNESS)
                    if roughness > 1:
                        roughness = 1
                    # apply weighting 
                    ti = (self.M1P*n_slope + self.M2P*roughness)*100
                    # apply platform specific UTC
                    # normalise ti such that if ti > UTC terrain is untraversable ie ti == 1 
                    # otherwise ti will be between 0 and 1 
                    ti = ti/self.UTC
                    if ti > 1:
                        ti = 1
                    self.t_map[row_index, col_index] = ti
                else:
                    # traversability is unknown
                    ti = -1
                    self.t_map[row_index, col_index] = ti

    def _roughness_metric(self, P, row_index, col_index):
        '''
            Evaluates the average plane fit residual over the mask centred about the given row,col indexes
            The equation of the plane P, should be fit over the window centred on the row,col index 
        '''
        # apply the mask
        [xl, yl, zl] = self._mask_cells(row_index, col_index)
        total_residual = 0
        for i in xrange(len(xl)):
            total_residual = total_residual + abs(zl[i] - z_value_plane(xl[i], yl[i], P))
        average_residual = total_residual/len(xl)
        return average_residual

    def _mask_cells(self, centre_row_index, centre_col_index):
        '''
            applies an L by L mask centred at the sepcified indexes and returns a list of points
            that represent the x,y position of a cell relative to the map frame. The z value of the 
            points returned will be set to the z_min of each cell (greatest depth in the elevation map)
            Note Z axis is towards the ground
        '''
        xl = []
        yl = []
        zl = []
        # determine indexs required to fit the LxL mask over the centre index
        mask_side_len = (self.mask_length - 1)/2
        start_row_index = centre_row_index - mask_side_len
        end_row_index =  centre_row_index + mask_side_len

        start_col_index = centre_col_index - mask_side_len
        end_col_index = centre_col_index + mask_side_len

        for row_index in xrange(start_row_index, end_row_index + 1):
            for col_index in xrange(start_col_index, end_col_index + 1):
                cell = self.emap.map[row_index, col_index]
                if cell.known == True:
                    # convert the cell centres into cartesian coordinates relative to the map frame
                    [x, y] = self.emap.index_to_cart(row_index, col_index)
                    z = cell.z_value_min
                    xl.append(x)
                    yl.append(y)
                    zl.append(z)
        return [xl, yl, zl]
