import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_threshold_updater/src/uwb_threshold_updater/data_structures')
# importing plane fitting functions from traversability analysis functions
sys.path.append('/home/spartan3123/catkin_ws/src/traversability_analysis/src/traversability_index/functions')
import numpy as np
from plane_fit import plane_fit
from plane_fit import slope_plane
from meta_cell import MetaCell
from utils_thesis.mapping import cartesian_2_index
import math as m

'''
 Given an Elevation map object, perform ground plane fitting 
 and store the surface gradients in a meta_data map. The meta data is used
 to color the pointcloud produced from the orginal elevation map object.
'''

class GroundMap(object):
    def __init__(self, emap, window_size=5):
        self.emap = emap
        self.delta_x = emap.delta_x
        self.delta_y = emap.delta_y
        self.cell_size = emap.cell_size
        ''' (elevation map coordinate frame)
        (x axis)
        ^
        |
        |
        *_________> (y axis)
        '''
        self.num_rows = emap.num_rows
        self.num_cols = emap.num_cols
        # coordinate of origin
        self.origin_x = emap.origin_x
        self.origin_y = emap.origin_y
        self.mask_length = window_size
        # initilise surface normal map 
        meta_map = [[MetaCell() for i in xrange(self.num_cols)] for j in xrange(self.num_rows)]
        self.m_map = np.array(meta_map)

    def ground_fit(self):
        # apply a mask over the elevation map, fitting planes to a LxL mask 
        for row_index in xrange(self.emap.map.shape[0]):
            for col_index in xrange (self.emap.map.shape[1]):
                if self.emap.map[row_index, col_index].known == True:
                    # get x,y,z values using a LxL mask that is centred at row_i,col_i 
                    # where the x,y values are the cell centres in cartesian coordinates 
                    # and the z value is the z_min of each cell
                    [xl, yl, zl] = self._mask_cells(row_index, col_index)
                    if len(xl) >= self.mask_length*self.mask_length*0.6:
                        # get the equation of the plane that fits the points xl, yl, zl
                        P = plane_fit(xl=xl, yl=yl, zl=zl)
                        # P = [A,B,C,D] coeficients of the equation of a plane
                        # Get the normal vector
                        N = P[0:3]
                        # determine the slope of the plane, angle between surface normal and a vector normal to the xy plane
                        slope = slope_plane(N)
                        if slope > m.pi/2:
                            slope = slope - m.pi
                        # write the meta data into the metat data map (slopes and normals)
                        self.m_map[row_index, col_index].set_metrics(slope=slope, normal=N)


    def _mask_cells(self, centre_row_index, centre_col_index):
        '''
            applies an L by L mask centred at the sepcified indexes and returns a list of points
            that represent the x,y position of a cell relative to the map frame. The z value of the 
            points returned will be set to the z_max of each cell (greatest depth in the elevation map)
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
                    # get the lowest z value for each cell 
                    z = cell.z_value_max
                    xl.append(x)
                    yl.append(y)
                    zl.append(z)
        return [xl, yl, zl]

    def get_normal(self, x, y):
        '''
            Given x and y in cartesian coordinates relative to the map coordinate system
            return the normal vector vector as a numpy array 

            if the normal vector cannot be found then a zero vector is returned
        '''
        map_properties = {'cell_size':self.cell_size, 'origin_x_i':self.emap.origin_x, 'origin_y_i':self.emap.origin_y}
        [row_index, col_index ] = cartesian_2_index(x,y,map_properties)

        if self.m_map[row_index, col_index].known == True:
            n_vector = self.m_map[row_index, col_index].normal
            N = np.array(n_vector)
        else:
            N = np.array([0.0, 0.0, 0.0])
        return N






