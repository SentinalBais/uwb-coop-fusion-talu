#!/usr/bin/env python
import rospy
import math as m
import tf 
from nav_msgs.msg import OccupancyGrid
from traversability_analysis.msg import Map

def cartesian_2_index(x, y, map_params):
    
    cell_size = map_params['cell_size']
    origin_x_i = map_params['origin_x_i']
    origin_y_i = map_params['origin_y_i']

    row_index = int(round(x/cell_size)) + origin_x_i
    col_index = int(round(y/cell_size)) + origin_y_i
    return [row_index, col_index]


def index_2_cartesian(row_index, col_index, map_params):

	cell_size = map_params['cell_size']
	origin_x_i = map_params['origin_x_i']
	origin_y_i = map_params['origin_y_i']

	x_value = row_index*cell_size - origin_x_i*cell_size
	y_value = col_index*cell_size - origin_y_i*cell_size
	return [x_value, y_value]

def gen_occupany_grid_msg(tmap, map_params):
    '''
        Generates a ROS occupancy grid msg for a tmap represented as a numpy array
    '''
    # constant convert degrees to radians
    DEG2RAD = m.pi/180.0
    omap = OccupancyGrid()                        
    numRows = tmap.shape[0]
    numCols = tmap.shape[1]
    omap.header.frame_id = "map"
    omap.info.map_load_time = rospy.get_rostime()
    omap.info.resolution = map_params['cell_size'] #meters
    omap.info.width = numCols  #
    omap.info.height = numRows # number of rows 

    ''' Convention for OccupancyGrid 
        Rows || to the x axis, ie the x axis indexes collumns
        Colls || to the y axis, ie the y axis indexes rows
    '''
    # determine the pose of the cell[0,0] in the map coordinate system
    [x, y] = index_2_cartesian(0,0, map_params)
    omap.info.origin.position.x = x
    omap.info.origin.position.y = y
    omap.info.origin.position.z = -20

    # http://mathworld.wolfram.com/EulerAngles.html
    temp = tf.transformations.quaternion_from_euler(180*DEG2RAD, 0*DEG2RAD, 90*DEG2RAD) #ax, ay, az 
    # where z rotation is applied first then y 
    omap.info.origin.orientation.x = temp[0]
    omap.info.origin.orientation.y = temp[1]
    omap.info.origin.orientation.z = temp[2]
    omap.info.origin.orientation.w = temp[3]
    #load data row-major iteation of obstacle map 
    for row in tmap.t_map:
        for ti in row:
            if ti < 0:  #unknown
                omap.data.append(-1)
            else:
                omap.data.append(int(ti*100))
    ''' Figuring out coordinate convention for OccupancyGrid msg
    for i in xrange(tmap.t_map.shape[0]):
        for j in xrange(tmap.t_map.shape[1]):
            if i == 1:
                omap.data.append(100)
            else:
                if j == 1:
                    omap.data.append(0)
                else:
                    omap.data.append(-1)
    '''
    return omap

def gen_occupany_grid_msg_np(tmap, map_params):
    '''
        Generates a ROS occupancy grid msg for a tmap represented as a numpy array
    '''
    # constant convert degrees to radians
    DEG2RAD = m.pi/180.0
    omap = OccupancyGrid()                        
    numRows = tmap.shape[0]
    numCols = tmap.shape[1]
    omap.header.frame_id = "map"
    omap.info.map_load_time = rospy.get_rostime()
    omap.info.resolution = map_params['cell_size'] #meters
    omap.info.width = numCols  #
    omap.info.height = numRows # number of rows 

    ''' Convention for OccupancyGrid 
        Rows || to the x axis, ie the x axis indexes collumns
        Colls || to the y axis, ie the y axis indexes rows
    '''
    # determine the pose of the cell[0,0] in the map coordinate system
    [x, y] = index_2_cartesian(0,0, map_params)
    omap.info.origin.position.x = x
    omap.info.origin.position.y = y
    omap.info.origin.position.z = 10

    # http://mathworld.wolfram.com/EulerAngles.html
    temp = tf.transformations.quaternion_from_euler(180*DEG2RAD, 0*DEG2RAD, 90*DEG2RAD) #ax, ay, az 
    # where z rotation is applied first then y 
    omap.info.origin.orientation.x = temp[0]
    omap.info.origin.orientation.y = temp[1]
    omap.info.origin.orientation.z = temp[2]
    omap.info.origin.orientation.w = temp[3]
    #load data row-major iteation of obstacle map 
    for row in tmap:
        for ti in row:
            if ti < 0:  #unknown
                omap.data.append(-1)
            else:
                omap.data.append(int(ti*100))
    ''' Figuring out coordinate convention for OccupancyGrid msg
    for i in xrange(tmap.t_map.shape[0]):
        for j in xrange(tmap.t_map.shape[1]):
            if i == 1:
                omap.data.append(100)
            else:
                if j == 1:
                    omap.data.append(0)
                else:
                    omap.data.append(-1)
    '''
    return omap

def gen_emap_msg(emap):
    map_msg = Map()
    map_msg.cell_size = emap.cell_size
    map_msg.origin_row_index = emap.origin_x
    map_msg.origin_col_index = emap.origin_y
    data = []
    # convert elevation map e_max into row major format 
    error_code = emap.z_max + 1
    # storing all elevations in cm 
    map_msg.error_code = int(error_code)
    map_msg.row_length = emap.num_cols
    # get the maxium z elevation map 
    emax = emap.getZ_max()
    for row in emax:
        for elevation in row:
            # data is stored in cm, to minimise rounding errors during int conversion 
            data.append(int(elevation*100))
    map_msg.data = data
    return map_msg